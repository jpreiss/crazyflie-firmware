/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * commander.c - the FreeRTOS wrapper around libcommander.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"
#include "libcommander.h"

#include "cf_math.h"
#include "param.h"
#include "static_mem.h"


/* Static state. */
static bool isInit;  // For CF init system.
static commander_t commander;  // Main state.

// Mutex is only used for high-level commands. Concurrency w/ radio task for
// low-level setpoints is handled exclusively through the queues.
static xSemaphoreHandle lockCmd;
static StaticSemaphore_t lockCmdBuffer;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

// Time of last setpoint-generating input. Not used in this module - only used
// so we can provide commanderGetInactivityTime() to power management system.
static uint32_t lastUpdate;


/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  const int priorityDisable = COMMANDER_PRIORITY_DISABLE;
  xQueueSend(priorityQueue, &priorityDisable, 0);

  libCommanderInit(
    &commander,
    COMMANDER_WDT_TIMEOUT_STABILIZE,
    COMMANDER_WDT_TIMEOUT_SHUTDOWN
  );
  lockCmd = xSemaphoreCreateMutexStatic(&lockCmdBuffer);
  ASSERT(lockCmd);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();
  isInit = true;
}

xSemaphoreHandle getCmdLock()
{
  return lockCmd;
}

commander_t *getCmd()
{
  return &commander;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
  }
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  static setpoint_t tempSetpoint;

  uint32_t millis = T2M(xTaskGetTickCount());

  if (xQueueReceive(setpointQueue, &tempSetpoint, 0) == pdTRUE) {
    libCommanderLowSetpoint(&commander, millis, &tempSetpoint);
    lastUpdate = millis;
  }

  libCommanderStep(&commander, millis, state, setpoint);
}

bool commanderTest(void)
{
  return isInit;
}

bool commanderIsIdle(void)
{
  return commander.mode == MODE_OFF_IDLE;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}
