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

#include "i2cdev.h"
#include "sleepus.h"
#include "task.h" // one function requires a CPU-yielding sleep

// the pca9685 uses 8-bit internal addresses.
enum Registers
{
  regMode1 = 0x00,
  regMode2,
  regSubAdr1,
  regSubAdr2,
  regSubAdr3,
  regAllCallAdr,
  // Each channel has 4 registers:
  // [ ON_L | ON_H | OFF_L | OFF_H ]
  regLED_FIRST = 0x06,
  regLED_LAST = 0x45,
  regLED_ALL = 0xFA,
  regPreScale = 0xFE,
  regTestMode = 0xFF,
};

int const LED_NBYTES = 4 * (regLED_LAST - regLED_FIRST + 1);

// the bits of the Mode1 register, as masks.
enum Mode1
{
   m1AllCall = 1 << 0,
      m1Sub3 = 1 << 1,
      m1Sub2 = 1 << 2,
      m1Sub1 = 1 << 3,
     m1Sleep = 1 << 4,
  m1AutoIncr = 1 << 5,
    m1ExtClk = 1 << 6,
   m1Restart = 1 << 7,
};

// TODO: mode2

// it always returns true, to allow chaining with && operator.
static inline bool sleepus2(uint32_t us)
{
  sleepus(us);
  return true;
}

static inline bool sleepms(uint32_t ms)
{
  return sleepus2(ms * 1000);
}

static inline bool sleepsec(uint32_t sec)
{
  return sleepms(sec * 1000);
}

static inline int channelReg(int channel)
{
  return regLED_FIRST + 4 * channel;
}

static inline int roundPositive(float x)
{
  return x + 0.5f;
}

static inline void u16ToByte(uint16_t i, uint8_t *bytes)
{
  bytes[0] = i & 0xFF;
  bytes[1] = i >> 8;
}

static void dutyToBytes(float duty, uint8_t *bytes)
{
  if (duty >= 1) {
    u16ToByte(4096, bytes);
    u16ToByte(0, bytes + 2);
  }
  else if (duty <= 0) {
    u16ToByte(0, bytes);
    u16ToByte(4096, bytes + 2);
  }
  else {
    u16ToByte(0, bytes);
    u16ToByte(duty * 4096, bytes + 2);
  }
}

//
//
// PUBLIC
//
//

bool initialize(int addr)
{
  if (!i2cdevInit(I2C1_DEV)) {
    return false;
  }

  // compared to default state, we:
  // - wake up from sleep
  // - enable AutoIncrement
  // - disable AllCall
  uint8_t settings = m1AutoIncr;
  return i2cdevWriteByte(&deckBus, addr, regMode1, settings);
}

// it will reset everything, can't pick which address
bool resetAll()
{
  // The reset sequence is the special "general call address" 0x00
  // followed by one byte 0x06, but no further data.
  // Technically, we shouldn't send the dontCare byte at all.
  // However, it should still work. It might return false...
  uint8_t dontCare = 0x00;
  return i2cdevWriteByte(&deckBus, 0x00, 0x06, dontCare);
}

bool goToSleep(int addr)
{
  return i2cdevWriteBit(&deckBus, addr, regMode1, 4, 1);
}

bool wakeUpForget(int addr)
{
  return i2cdevWriteBit(&deckBus, addr, regMode1, 4, 0);
}

bool wakeUpRestore(int addr)
{
  uint8_t restorable = 0x00;
  if (!i2cdevReadBit(&deckBus, addr, regMode1, m1Restart, &restorable)) {
    // TODO: should it wake up anyway?
    return false;
  }
  if (restorable != 0x00 && wakeUpForget(addr)) {
    vTaskDelay(F2T(1000)); // datasheet calls for >= 500us, being careful
    return i2cdevWriteBit(&deckBus, addr, regMode1, m1Restart, 1);
  }
  return false;
}

// This should be used in preference to multiple setChannelDuty() calls.
// It uses the i2c bus more efficiently.
bool setMultiChannelDuty(int addr, int chanBegin, int nChan, float const *duties)
{
  uint8_t data[LED_NBYTES];
  for (int i = 0; i < nChan; ++i) {
    dutyToBytes(duties[i], data + 4*i);
  }
  int const reg = channelReg(chanBegin);
  return i2cdevWrite(&deckBus, addr, reg, 4*nChan, data);
}

// TODO phase
bool setChannelDuty(int addr, int channel, float duty)
{
  return setMultiChannelDuty(addr, channel, 1, &duty);
}

bool setPwmFrequency(int addr, float freq)
{
  static float const OSC_CLOCK = 25.0f * 1000.0f * 1000.0f;
  int const prescale = roundPositive(OSC_CLOCK / (4096.0f * freq)) - 1;
  return
    (prescale < 0x03 || prescale > 0xFF) &&
    goToSleep(addr) &&
    i2cdevWriteByte(&deckBus, addr, regPreScale, (uint8_t)prescale) &&
    wakeUpRestore(addr);
}
