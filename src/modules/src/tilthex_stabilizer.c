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
#include "usec_time.h"

#define min(a,b) ((b)<(a)?(b):(a))

static bool isInit = false;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static float thrusts[6];
static uint16_t pwm_durations[6];
static uint8_t viconFresh = false;

static void tilthexStabilizerTask(void* param);

static int const I2C_ADDR = 0x40;
static int const ESC_PWM_FREQ = 385;

static uint16_t usec_kalman;
static uint16_t usec_control;
static uint16_t usec_total;


static uint16_t omegaToDuration(float omega)
{
  // experimentally determined with Afro 2-amp ESC
  // values are in "pca9685 units" so depend on PWM_ESC_FREQ
  // (set to 385Hz in the experiments, should be same here.)
  uint16_t duration = 0.4f * (omega + 3683);
  if (duration > 2400) duration = 2400;
  if (duration < 1700) duration = 1700;
  return duration;
}

static bool tilthexPowerDistribution(float const omega2[6])
{
  for (int i = 0; i < 6; ++i) {
    float omega = sqrtf(omega2[i]);
    pwm_durations[i] = omegaToDuration(omega);
  }
  return pca9685setDurationsAsync(I2C_ADDR, 0, 6, pwm_durations);
}

static void tilthexPowerStop()
{
  float x[6] = { 0.0f, };
  tilthexPowerDistribution(x);
}


void tilthexStabilizerInit(StateEstimatorType estimator)
{
  for (int i = 0; i < 6; ++i) {
    thrusts[i] = i;
  }

  if (isInit) {
    return;
  }

  sensorsInit();
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif
  initUsecTimer();

  if (!pca9685init(I2C_ADDR, ESC_PWM_FREQ)) {
    return;
  }
  if (!pca9685startAsyncTask()) {
    return;
  }

  tilthexPowerStop();

  xTaskCreate(tilthexStabilizerTask, TILTHEX_STABILIZER_TASK_NAME,
              TILTHEX_STABILIZER_TASK_STACKSIZE, NULL, TILTHEX_STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool tilthexStabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
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

static struct mat33 quat2mathrot(struct quaternion_s q)
{
  struct quat quat = { .x = q.x, .y = q.y, .z = q.z, .w = q.w };
  struct mat33 R = quat2rotmat(quat);
  return R;
}


bool sleepsec(float sec)
{
  vTaskDelay(F2T(1.0f / sec));
  return true;
}

static bool test9685()
{
  int sleeptime = 1;

  // arm ESCs
  int const N_DUTIES = 10;
  float duties[N_DUTIES];
  for (int i = 0; i < N_DUTIES; ++i) {
    duties[i] = 0.4;
  }

  bool val =

  pca9685setDuties(I2C_ADDR, 0, N_DUTIES, duties) &&

  sleepsec(0.5) &&

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

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    uint64_t tick_begin = usecTimestamp();

    // tell EKF that we are flying
    control.roll = 0;
    control.pitch = 0;
    control.yaw = 0;
    control.thrust = 9.81;

    if (getExtPosition(&state)) {
      stateEstimator(&state, &sensorData, &control, tick);
    }

    uint64_t tick_kalman = usecTimestamp();
    usec_kalman = min(tick_kalman - tick_begin, UINT16_MAX);

    commanderGetSetpoint(&setpoint, &state);
    //sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
    //stateController(&control, &setpoint, &sensorData, &state, tick);

    struct tilthex_state s;
    struct tilthex_state des;

    // diagnostic / testing modes

#define REAL_LIFE

#if defined(HOLD_SETPOINT)
    // pretend we're already at the setpoint, and hold it
    des.pos = vec2math(setpoint.position);
    des.vel = vec2math(setpoint.velocity);
    des.acc = vec2math(setpoint.acceleration);
    des.omega = attitude2math(setpoint.attitudeRate);
    des.R = quat2mathrot(setpoint.attitudeQuaternion);
    s = des;
#elif defined(HOLD_ATTITUDE)
    // pretend the setpoint is pos=0 with current attitude.
    // for state, use real attitude but pretend pos, vel are 0.
    s.pos = vzero();
    s.vel = vzero();
    s.acc = vzero();
    s.omega = vzero();
    s.R = quat2mathrot(state.attitudeQuaternion);
    des = s;
#elif defined(REAL_LIFE)
    des.pos = vec2math(setpoint.position);
    des.vel = vec2math(setpoint.velocity);
    des.acc = vec2math(setpoint.acceleration);
    des.omega = attitude2math(setpoint.attitudeRate);
    des.R = quat2mathrot(setpoint.attitudeQuaternion);

    // correct from Crazyflie's left handed coords into x fwd, y left, z up
    s.pos = vec2math(state.position);
    s.pos.y = -s.pos.y;
    s.vel = vec2math(state.velocity);
    s.vel.y = -s.vel.y;
    s.acc = vec2math(state.acc);
    s.acc.y = -s.acc.y;
    s.omega = attitude2math(state.attitudeRate);
    s.omega.y = -s.omega.y;
    // no y correction needed for quat, I think
    s.R = quat2mathrot(state.attitudeQuaternion);
#else
  #error "no tilthex control mode set."
#endif

    tilthex_control(s, des, thrusts);

    uint64_t tick_control = usecTimestamp();
    usec_control = min(tick_control - tick_kalman, UINT16_MAX);

    tilthexPowerDistribution(thrusts);

    //checkEmergencyStopTimeout();

    //if (emergencyStop) {
      //tilthexPowerStop();
    //} else {
      //tilthexPowerDistribution(thrusts);
    //}
    uint64_t tick_end = usecTimestamp();
    usec_total = min(tick_end - tick_begin, UINT16_MAX);

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
LOG_GROUP_START(setpointAcc)
LOG_ADD(LOG_FLOAT, x, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.acceleration.z)
LOG_GROUP_STOP(setpointAcc)

LOG_GROUP_START(vicon)
LOG_ADD(LOG_UINT8, fresh, &viconFresh)
LOG_GROUP_STOP(vicon)

LOG_GROUP_START(tilthexPWM)
LOG_ADD(LOG_UINT16, p0, &pwm_durations[0])
LOG_ADD(LOG_UINT16, p1, &pwm_durations[1])
LOG_ADD(LOG_UINT16, p2, &pwm_durations[2])
LOG_ADD(LOG_UINT16, p3, &pwm_durations[3])
LOG_ADD(LOG_UINT16, p4, &pwm_durations[4])
LOG_ADD(LOG_UINT16, p5, &pwm_durations[5])
LOG_GROUP_STOP(tilthexPWM)

LOG_GROUP_START(thProfile)
LOG_ADD(LOG_UINT16, uskalman, &usec_kalman)
LOG_ADD(LOG_UINT16, uscontrol, &usec_control)
LOG_ADD(LOG_UINT16, ustotal, &usec_total)
LOG_GROUP_STOP(thProfile)

