/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * pca9685.c: 12-bit, 16-channel PWM servo (, LED, ESC, ...) driver
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include "pca9685.h"
#include "pca9685async.h"

struct asyncRequest
{
  int addr;
  int chanBegin;
  int nChan;
  float duties[6];
};

static struct asyncRequest reqPush;
static struct asyncRequest reqPop;

static TaskHandle_t task;

static QueueHandle_t queue;

static TickType_t const PUSH_WAIT_TICKS = 1; // 1 ms - TODO use units, macros

static void asyncTask(__attribute__((unused)) void *param)
{
  while (true) {
    BaseType_t ok = xQueueReceive(queue, &reqPop, portMAX_DELAY);
    if (ok == pdTRUE) {
      // blocking message send.
      pca9685setMultiChannelDuty(
        reqPop.addr, reqPop.chanBegin, reqPop.nChan, reqPop.duties);
    }
  }
}

bool pca9685startAsyncTask()
{
  queue = xQueueCreate(1, sizeof(struct asyncRequest));
  if (queue == 0) {
    return false;
  }

  BaseType_t xReturned = xTaskCreate(
    &asyncTask,
    PCA9685_TASK_NAME,
    PCA9685_TASK_STACKSIZE,
    NULL,
    PCA9685_TASK_PRI - 1,
    &task);

  return xReturned == pdPASS;
}

bool pca9685setMultiChannelDutyAsync(
  int addr, int chanBegin, int nChan, float const *duties)
{
  reqPush.addr = addr;
  reqPush.chanBegin = chanBegin;
  reqPush.nChan = nChan;
  for (int i = 0; i < nChan; ++i) {
    reqPush.duties[i] = duties[i];
  }
  // don't wait ever. TODO: is it what we want?
  BaseType_t sent = xQueueSend(queue, &reqPush, 0);
  return true;
}
