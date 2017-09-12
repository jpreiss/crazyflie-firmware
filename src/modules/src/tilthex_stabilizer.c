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

#include "ekf.h"
#include "mathconstants.h"
#include "tilthex_control.h"
#include "tilthex_song.h"
#include "pca9685.h"
#include "usec_time.h"

static bool isInit = false;
static bool emergencyStop = false;
static int emergencyStopTimeout = 0;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static float thrusts[6];

static void tilthexStabilizerTask(void* param);

static int const I2C_ADDR = 0x40;
static int const ESC_PWM_FREQ = 385;

// EKF implementation uses double-buffered approach
static struct ekf ekfa;
static struct ekf ekfb;
static struct ekf *ekf_front = &ekfa;
static struct ekf *ekf_back = &ekfb;
static void ekf_flip()
{
  struct ekf *ekf_temp = ekf_front;
  ekf_front = ekf_back;
  ekf_back = ekf_temp;
}
static bool first_vicon = false;

// logging
static struct vec ekf_pos;
static struct vec ekf_rpy;
static struct vec setpoint_rpy;
static float ekf_usec = 0;

static float logHaveSetpoint = 0;
static float logHaveVicon = 0;
static float logEmergencyStop = 0;
static float x_frac = -1.0f;


void tilthexStabilizerInit(StateEstimatorType estimator)
{
  for (int i = 0; i < 6; ++i) {
    thrusts[i] = i;
  }

  if (isInit) {
    return;
  }

  sensorsInit();
  /*
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif
  */

  if (!pca9685init(I2C_ADDR, ESC_PWM_FREQ)) {
    return;
  }

  // Initialize to 0 so gyro integration still works without Vicon
  float init[] = {0, 0, 0, 1};
  ekf_init(ekf_back, init, init, init);

  xTaskCreate(tilthexStabilizerTask, TILTHEX_STABILIZER_TASK_NAME,
              TILTHEX_STABILIZER_TASK_STACKSIZE, NULL, TILTHEX_STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool tilthexStabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
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


static uint16_t omegaToDuration(float omega)
{
  // experimentally determined with Afro 2-amp ESC
  // values are in "pca9685 units" so depend on PWM_ESC_FREQ
  // (set to 385Hz in the experiments, should be same here.)
  uint16_t duration = 0.959f * omega + 1126.1f;
  if (duration > 3200) duration = 3200;
  if (duration < 1500) duration = 1500;
  return duration;
}

static bool tilthexPowerDistribution(float const omega2[6])
{
  uint16_t duration[6];
  for (int i = 0; i < 6; ++i) {
    float omega = sqrtf(omega2[i]);
    duration[i] = omegaToDuration(omega);
  }
  return pca9685setDurationsAsync(I2C_ADDR, 0, 6, duration);
}

static bool tilthexDurationTest(uint16_t duration)
{
  uint16_t durations[6];
  for (int i = 0; i < 6; ++i) {
    durations[i] = duration;
  }
  return pca9685setDurationsAsync(I2C_ADDR, 0, 6, durations);
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

  // arm ESCs
  int const N_DUTIES = 10;
  float duties[N_DUTIES];
  for (int i = 0; i < N_DUTIES; ++i) {
    duties[i] = 0.3;
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

static int const TIMEOUT_TICKS = 100; // 0.2 sec

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

  if (!pca9685startAsyncTask()) {
    return;
  }

  sleepsec(0.1);

  // the "0 thrust" signal is still nonzero pulsewidth.
  // this arms the Afro ESCs.
  tilthexPowerStop();

  //songBegin(I2C_ADDR);

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));


    bool gotPose = getExtPose(&state);
    //stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);
    //sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
    //stateController(&control, &setpoint, &sensorData, &state, tick);

    //songStep(tick);

    uint64_t const ekf_tic = usecTimestamp();

    // lazy initialization
    if (!first_vicon && gotPose) {
      float const vel[3] = {0, 0, 0};
      ekf_init(ekf_back,
        (float const *)&state.position.x,
        vel,
        (float const *)&state.attitudeQuaternion.x);
      first_vicon = true;
    }

    if (first_vicon && sensorsReadAcc(&sensorData.acc) && sensorsReadGyro(&sensorData.gyro)) {
      float acc[3] = {sensorData.acc.x * GRAV, sensorData.acc.y * GRAV, sensorData.acc.z * GRAV};
      float gyro[3] = {radians(sensorData.gyro.x), radians(sensorData.gyro.y), radians(sensorData.gyro.z)};
      ekf_imu(ekf_back, ekf_front, acc, gyro, 1.0f/500.0f); // TODO not hard-code!!!!
      ekf_flip();
    }

    // logging
    ekf_rpy = quat2rpy(ekf_back->quat);
    ekf_pos = ekf_back->pos;

    if (gotPose) {
      float pos_vicon[3] = {state.position.x, state.position.y, state.position.z};
      ekf_vicon(ekf_back, ekf_front, pos_vicon, &state.attitudeQuaternion.x);
      ekf_flip();
      emergencyStop = false;
      emergencyStopTimeout = TIMEOUT_TICKS;
    }

    if (emergencyStopTimeout >= 0) {
      emergencyStopTimeout -= 1;

      if (emergencyStopTimeout == 0) {
        emergencyStop = true;
      }
    }

    uint64_t const ekf_toc = usecTimestamp();
    ekf_usec = ekf_toc - ekf_tic;

    struct tilthex_state s;
    struct tilthex_state des;

#define TRACK_SETPOINT

#if defined(HOLD_ATTITUDE)
    s.pos = vzero();
    s.vel = vzero();
    s.acc = vzero();
    s.omega = vzero();
    s.R = quat2rotmat(ekf_back->quat);

    des = s;

#elif defined(TRACK_SETPOINT)
    s.pos = ekf_back->pos;
    s.vel = ekf_back->vel;
    s.acc = ekf_back->acc;
    s.omega = ekf_back->omega;
    s.R = quat2rotmat(ekf_back->quat);

    //des = s;
    des.pos = vec2math(setpoint.position);
    des.vel = vec2math(setpoint.velocity);
    des.acc = vec2math(setpoint.acceleration);
    des.omega = attitude2math(setpoint.attitudeRate);
    struct quat setpoint_quat = quat2math(setpoint.attitudeQuaternion);
    des.R = quat2rotmat(setpoint_quat);
#else
    #error "tilthex_stabilizer - no control mode specified!"
#endif

    setpoint_rpy = quat2rpy(setpoint_quat);

    tilthex_control(s, des, thrusts);

    bool haveSetpoint = commanderGetInactivityTime() < 100;

    logHaveSetpoint = haveSetpoint;
    logHaveVicon = first_vicon;
    logEmergencyStop = emergencyStop;

    if (true) {
    //if (songIsDone()) {
      if (first_vicon && !emergencyStop && haveSetpoint) {
        // DEBUG HACK for rpm calib
        /*
        x_frac = setpoint.position.x;
        if (x_frac >= 0 && x_frac <= 1) {
          float omega = x_frac * 2300;
          for (int i = 0; i < 6; ++i) {
            thrusts[i] = omega * omega;
          }
          tilthexPowerDistribution(thrusts);
        }
        //uint16_t duration = setpoint.position.x * 1000.0f;
        //tilthexDurationTest(duration);
        */
        tilthexPowerDistribution(thrusts);
      }
      else {
        tilthexPowerStop();
      }
    }

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

void tilthexStabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}
*/

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

LOG_GROUP_START(tiltDbg)
LOG_ADD(LOG_FLOAT, vicon, &logHaveVicon)
LOG_ADD(LOG_FLOAT, setpt, &logHaveSetpoint)
LOG_ADD(LOG_FLOAT, emerg, &logEmergencyStop)
LOG_ADD(LOG_FLOAT, x_frac, &x_frac)
LOG_GROUP_STOP(tiltDbg)


LOG_GROUP_START(tiltPosCtrl)
LOG_ADD(LOG_FLOAT, setx, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, sety, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, setz, &setpoint.position.z)
LOG_ADD(LOG_FLOAT, setvx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, setvy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, setvz, &setpoint.velocity.z)
LOG_ADD(LOG_FLOAT, roll, &setpoint_rpy.x)
LOG_ADD(LOG_FLOAT, pitch, &setpoint_rpy.y)
LOG_ADD(LOG_FLOAT, yaw, &setpoint_rpy.z)
LOG_GROUP_STOP(tiltPosCtrl)

LOG_GROUP_START(ekf_pos)
LOG_ADD(LOG_FLOAT, x, &ekf_pos.x)
LOG_ADD(LOG_FLOAT, y, &ekf_pos.y)
LOG_ADD(LOG_FLOAT, z, &ekf_pos.z)
LOG_ADD(LOG_FLOAT, usec, &ekf_usec)
LOG_ADD(LOG_FLOAT, vicon_x, &state.position.x)
LOG_ADD(LOG_FLOAT, vicon_y, &state.position.y)
LOG_ADD(LOG_FLOAT, vicon_z, &state.position.z)
LOG_GROUP_STOP(ekf_pos)

LOG_GROUP_START(ekf_rpy)
LOG_ADD(LOG_FLOAT, x, &ekf_rpy.x)
LOG_ADD(LOG_FLOAT, y, &ekf_rpy.y)
LOG_ADD(LOG_FLOAT, z, &ekf_rpy.z)
LOG_GROUP_STOP(ekf_rpy)
