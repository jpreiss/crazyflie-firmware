/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
#include <stdbool.h>

#include "crtp_commander.h"

#include "cfassert.h"
#include "commander.h"
#include "crtp.h"


static bool isInit;

static void commanderCrtpCB(CRTPPacket* pk);

void crtpCommanderInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
  isInit = true;
}

/* Channel 1 of the generic commander port is used for "meta-commands"
 * that alter the behavior of the commander itself, e.g. mode switching.
 * Although we use the generic commander port due to increasing pressure on the
 * 4-bit space of ports numbers, meta-commands that are unrelated to
 * streaming generic setpoint control modes are permitted.
 *
 * The packet format for meta-commands is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * TYPE is an 8-bit value. The remainder of the data depends on the command.
 * The maximum data size is 29 bytes.
 */


enum metaCommand {
  metaNotifySetpointsStop = 0,
  nMetaCommands,
};


typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);


struct notifySetpointsStopPacket {
  // See commander.h for description of behavior.
  uint32_t remainValidMillisecs;
} __attribute__((packed));


void notifySetpointsStopDecoder(const void *data, size_t datalen)
{
  ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
  const struct notifySetpointsStopPacket *values = data;
  commanderNotifySetpointsStop(values->remainValidMillisecs);
}


const static metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};


static void commanderCrtpCB(CRTPPacket* pk)
{
  static setpoint_t setpoint;

  if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
    crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
    switch (pk->channel) {
    case 0:
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      break;
    case 1:
      uint8_t metaCmd = pk->data[0];
      if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
        metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
      }
      break;
    }
  }
}
