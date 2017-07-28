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


// start the device. All channels will be initialized to 0% duty.
bool initialize(int addr);

// set all signals to 0 and go into low-power mode.
// call wakeUpRestore() to wake up with same PWM values.
bool goToSleep(int addr);

// wake up from sleep and revert to 0% duty for all channels.
bool wakeUpForget(int addr);

// wake up from sleep and restore the PWM settings from before sleeping.
// will not work if any other functions are called in between sleep and wake.
bool wakeUpRestore(int addr);

// set duty cycle of several channels at once.
// this should be used in preference to multiple setChannelDuty() calls.
// it uses the i2c bus more efficiently.
bool setMultiChannelDuty(int addr, int chanBegin, int nChan, float const *duties);

// set the duty cycle of one channel.
bool setChannelDuty(int addr, int channel, float duty);

// set the PWM frequency. it's the same for all channels.
// note: this function sleeps for about 500us as required by the hardware.
bool setPwmFrequency(int addr, float freq);

// for debugging only!!!
bool bigTest(int addr);
