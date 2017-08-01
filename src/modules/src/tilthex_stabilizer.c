/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

#include "tilthex_control.h"
#include "pca9685.h"

static bool isInit = false;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static float thrusts[6];

static void tilthexStabilizerTask(void* param);

static int const I2C_ADDR = 0x40;
static int const ESC_PWM_FREQ = 1000;


void tilthexStabilizerInit(StateEstimatorType estimator)
{
  for (int i = 0; i < 6; ++i) {
    thrusts[i] = i;
  }

  if (isInit) {
    return;
  }

  /*
  sensorsInit();
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif
  */

  // I2C
  bool ok =
    pca9685init(I2C_ADDR) &&
    pca9685setPwmFreq(I2C_ADDR, ESC_PWM_FREQ);
  if (!ok) {
    return;
  }


  xTaskCreate(tilthexStabilizerTask, TILTHEX_STABILIZER_TASK_NAME,
              TILTHEX_STABILIZER_TASK_STACKSIZE, NULL, TILTHEX_STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool tilthexStabilizerTest(void)
{
  bool pass = true;

  //pass &= sensorsTest();
  //pass &= stateEstimatorTest();
  //pass &= stateControllerTest();
  //pass &= powerDistributionTest();

  return pass;
}

/*
static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}
*/

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

/*
static struct vec3_s math2vec(struct vec v)
{
  struct vec3_s v3s = { .x = v.x, .y = v.y, .z = v.z };
  return v3s;
}
*/

static struct vec vec2math(struct vec3_s v)
{
  struct vec vec = { .x = v.x, .y = v.y, .z = v.z };
  return vec;
}

static struct vec attitude2math(struct attitude_s a)
{
  struct vec vec = { .x = a.roll, .y = a.pitch, .z = a.yaw };
  return vec;
}

static struct quat quat2math(struct quaternion_s q)
{
  struct quat quat = { .x = q.x, .y = q.y, .z = q.z, .w = q.w };
  return quat;
}

// TODO these values
#define ESC_DUTY_MAX 0.9f
#define ESC_DUTY_MIN 0.2f
#define ESC_DUTY_RANGE (ESC_DUTY_MAX - ESC_DUTY_MIN)
#define RPM_MAX 20000

#define OMEGA_MAX ((float)(2.0 * M_PI * RPM_MAX / 60.0))
#define OMEGA_TO_DUTY (ESC_DUTY_RANGE / OMEGA_MAX)

static bool tilthexPowerDistribution(float const omega2[6])
{
  float duty[6];
  for (int i = 0; i < 6; ++i) {
    float omega = sqrtf(omega2[i]);
    duty[i] = OMEGA_TO_DUTY * omega + ESC_DUTY_MIN;
  }
  return pca9685setMultiChannelDuty(I2C_ADDR, 0, 6, duty);
}

static void tilthexPowerStop()
{
  float x[6] = { 0.0f, };
  tilthexPowerDistribution(x);
}


bool sleepsec(float sec)
{
  vTaskDelay(F2T(1.0f / sec));
  return true;
}

static bool test9685()
{
  int sleeptime = 1;

  int const N_DUTIES = 10;
  float duties[N_DUTIES];
  for (int i = 0; i < N_DUTIES; ++i) {
    duties[i] = i / (N_DUTIES - 1.0f);
  }

  bool val =

  pca9685setMultiChannelDuty(I2C_ADDR, 0, N_DUTIES, duties);
  //pca9685setChannelDuty(I2C_ADDR, 0, 0.5) &&

  //pca9685setChannelDuty(I2C_ADDR, 1, 0.1) &&

  /*
  sleepsec() &&

  pca9685setChannelDuty(I2C_ADDR, 0, 0.9) &&
  sleepsec(sleeptime) &&

  pca9685sleep(I2C_ADDR) &&
  sleepsec(sleeptime) &&

  pca9685wakeForget(I2C_ADDR) &&
  sleepsec(sleeptime) &&

  pca9685setPwmFreq(I2C_ADDR, 200) &&
  sleepsec(2*sleeptime) &&

  pca9685setPwmFreq(I2C_ADDR, 500) &&
  sleepsec(2*sleeptime) &&

  pca9685setPwmFreq(I2C_ADDR, 1000) &&
  sleepsec(2*sleeptime) &&
  */

  true;

  return val;
}

static void tilthexStabilizerTask(void* param)
{
  uint32_t tick = 1;
  vTaskSetApplicationTaskTag(0, (void*)TASK_TILTHEX_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  uint32_t lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  if (!test9685()) {
    return;
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);

    //commanderGetSetpoint(&setpoint, &state);
    //sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
    //stateController(&control, &setpoint, &sensorData, &state, tick);

    struct tilthex_state s;
    //s.pos = vec2math(state.position);
    s.pos = vzero();
    //s.vel = vec2math(state.velocity);
    s.vel = vzero();
    s.acc = vec2math(state.acc);
    s.omega = attitude2math(state.attitudeRate);
    s.R = quat2rotmat(quat2math(state.attitudeQuaternion));

    struct tilthex_state des;
    //des.pos = vec2math(setpoint.position);
    des.pos = vzero();
    //des.vel = vec2math(setpoint.velocity);
    des.vel = vzero();
    //des.acc = vec2math(setpoint.acceleration);
    des.acc = vzero();
    //des.omega = attitude2math(setpoint.attitudeRate);
    des.omega = vzero();
    //des.R = quat2rotmat(quat2math(setpoint.attitudeQuaternion));
    des.R = eye();

    tilthex_control(s, des, thrusts);
    tilthexPowerDistribution(thrusts);

    //checkEmergencyStopTimeout();

    //if (emergencyStop) {
      //tilthexPowerStop();
    //} else {
      //tilthexPowerDistribution(thrusts);
    //}

    tick++;
  }
}

/*
void tilthexStabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void tilthexStabilizerResetEmergencyStop()
{
  emergencyStop = false;
}
*/

void tilthexStabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_GROUP_STOP(stateEstimate)
*/

// TiltHex parts
LOG_GROUP_START(tilthexThrusts)
LOG_ADD(LOG_FLOAT, t0, &thrusts[0])
LOG_ADD(LOG_FLOAT, t1, &thrusts[1])
LOG_ADD(LOG_FLOAT, t2, &thrusts[2])
LOG_ADD(LOG_FLOAT, t3, &thrusts[3])
LOG_ADD(LOG_FLOAT, t4, &thrusts[4])
LOG_ADD(LOG_FLOAT, t5, &thrusts[5])
LOG_GROUP_STOP(tilthexThrusts)

