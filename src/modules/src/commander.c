/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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

/* commander.c - top level commander that handles high/low-level switching,
 * preemption, emergencies, etc. Core state machine functionality comes from
 * libcommander.c. This file adds input queues (because the radio tasks are
 * asynchronous w.r.t. the stabilizer) and all other direct interfacing with
 * FreeRTOS.
 */

// Standard value for how long non-stabilizer threads (e.g. radio) should wait
// to insert an event in the queue before giving up and dropping the command.
#define EVENT_QUEUE_BLOCK_TICKS (M2T(500))

/* Definitions. */
// Event queue tagged union.
enum cmdEvent
{
  EVENT_HIGH_LEVEL_RECVD,
  EVENT_NOTIFY_SETPOINTS_STOP,
};

typedef struct commander_event_s
{
  enum cmdEvent tag;
  union {
    uint32_t notifySetpointsStopMillis;
  };
} commander_event_t;


/* Static state. */
static bool isInit;  // For CF init system.
static commander_t commander;  // Main state.

// Time of last setpoint-generating input. Not used in this module - only used
// so we can provide commanderGetInactivityTime() to power management system.
static uint32_t lastUpdate;

// Setpoint and event queues.
// From the perspective of libcommander a low-level setpoint is just another
// event, but we don't put it in the same queue as other events. We use a
// one-item queue for setpoints because there is no reason to care about old
// setpoints once a newer one has arrived. In contrast, high-level commands
// must not be discarded.
static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));
// TODO: In a perfect world this queue would be unbounded - is 4 enough?
static QueueHandle_t eventQueue;
STATIC_MEM_QUEUE_ALLOC(eventQueue, 4, sizeof(commander_event_t));


/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  const int priorityDisable = COMMANDER_PRIORITY_DISABLE;
  xQueueSend(priorityQueue, &priorityDisable, 0);

  eventQueue = STATIC_MEM_QUEUE_CREATE(eventQueue);
  ASSERT(eventQueue);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();
  libCommanderInit(
    &commander,
    COMMANDER_WDT_TIMEOUT_STABILIZE,
    COMMANDER_WDT_TIMEOUT_SHUTDOWN
  );

  isInit = true;
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

void commanderNotifySetpointsStop(int remainValidMillisecs)
{
  commander_event_t event = {
    .tag = EVENT_NOTIFY_SETPOINTS_STOP,
    .notifySetpointsStopMillis = remainValidMillisecs,
  };
  xQueueSend(eventQueue, &event, EVENT_QUEUE_BLOCK_TICKS);
  // TODO: Handle queue full condition other than dropping command?
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  static commander_event_t tempEvent;
  static setpoint_t tempSetpoint;

  uint32_t ticks = xTaskGetTickCount();

  // Process all events that were enqueued since the last stabilizer loop.
  while (xQueueReceive(eventQueue, &tempEvent, 0) == pdTRUE) {
    switch (tempEvent.tag) {
    case EVENT_NOTIFY_SETPOINTS_STOP:
      libCommanderNotifySetpointsStop(&commander, ticks, tempEvent.notifySetpointsStopMillis);
      break;
    case EVENT_HIGH_LEVEL_RECVD:
      libCommanderHighLevelRecvd(&commander, ticks);
      lastUpdate = ticks;
      break;
    default:
      // TODO: error!
      break;
    }
  }
  if (xQueueReceive(setpointQueue, &tempSetpoint, 0) == pdTRUE) {
    libCommanderLowSetpoint(&commander, ticks, &tempSetpoint);
    lastUpdate = ticks;
  }

  libCommanderStep(&commander, ticks, state, setpoint);
}

void commanderTellHighLevelCmdRecvd()
{
  commander_event_t event = {
    .tag = EVENT_HIGH_LEVEL_RECVD,
  };
  xQueueSend(eventQueue, &event, EVENT_QUEUE_BLOCK_TICKS);
  // TODO: Handle queue full condition other than dropping command?
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
