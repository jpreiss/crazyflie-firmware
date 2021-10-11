#include "libcommander.h"

// TODO: Move high-level commander functionality into library-like architecture
// too, so the full commander framework can be compiled for x86.
#include "crtp_commander_high_level.h"

/* libcommander.c - Top-level state machine for Crazyflie.
 *
 * Intended to be compilable on x86 in the future. Currently it is not because
 * the high-level commander is left as a separate file with global/static state
 * and ARM / FreeRTOS dependencies. Eventually the platform-independent parts
 * of the high-level commander should be moved here.
 *
 * NOTE: All switches over commander_t->mode are implemented WITHOUT a
 * "default" case so we get the compiler's help ensuring that we have handled
 * all possible modes.
 */

// Static structs are zero-initialized, so nullSetpoint corresponds to
// modeDisable for all stab_mode_t members and zero for all physical values.
// In other words, the controller should cut power upon recieving it.
const static setpoint_t nullSetpoint;


void libCommanderInit(commander_t *cmd, uint32_t levelingTimeout, uint32_t emergencyTimeout)
{
  cmd->mode = MODE_OFF_IDLE;
  cmd->levelingTimeout = levelingTimeout;
  cmd->emergencyTimeout = emergencyTimeout;
}

void libCommanderLowSetpoint(commander_t *cmd, uint32_t ticks, setpoint_t const *setpoint)
{
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_LOW:
  case MODE_LOW_LEVELING:
  case MODE_HIGH:
    cmd->lowSetpoint = *setpoint;
    cmd->mode = MODE_LOW;
    break;
  case MODE_OFF_EMERGENCY:
    // TODO: What should happen?
    break;
  case MODE_LOW_AWAITING_HIGH:
    // TODO: What should happen?
    break;
  }
}

void libCommanderNotifySetpointsStop(commander_t *cmd, uint32_t ticks, uint32_t awaitMillis)
{
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
  case MODE_HIGH:
    // Command does not make sense or is redundant.
    break;
  case MODE_LOW:
    cmd->awaitHighLevelTimeout = awaitMillis;
    cmd->mode = MODE_LOW_AWAITING_HIGH;
    // TODO: This is a global variable access!! In an ideal design the
    // high-level commander state would be a part of the commander_t.
    crtpCommanderHighLevelTellState(&cmd->lastState);
    break;
  case MODE_LOW_LEVELING:
    // TODO: What should happen?
    break;
  case MODE_LOW_AWAITING_HIGH:
    // TODO: How to reconcile old and new await timers?
    break;
  }
}

void libCommanderHighLevelRecvd(commander_t *cmd, uint32_t ticks)
{
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_LOW_AWAITING_HIGH:
  case MODE_HIGH:
    cmd->mode = MODE_HIGH;
  case MODE_OFF_EMERGENCY:
  case MODE_LOW:
  case MODE_LOW_LEVELING:
    // Do nothing - overridden!
    break;
  }
}

void libCommanderStep(commander_t *cmd, uint32_t ticks, state_t const *state, setpoint_t *setpointOut)
{
  // First pass: Mode changes due to passage of time.
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
    break;
  case MODE_LOW:
    if ((ticks - cmd->lowSetpoint.timestamp) > cmd->levelingTimeout) {
      cmd->mode = MODE_LOW_LEVELING;
    }
    // Technically we could fall through to MODE_LOW_LEVELING in case we passed
    // both timeouts in the same step, but this should never happen with
    // reasonable timeout values and stabilizer loop rate.
    break;
  case MODE_LOW_LEVELING:
    if ((ticks - cmd->lowSetpoint.timestamp) > cmd->emergencyTimeout) {
      cmd->mode = MODE_OFF_EMERGENCY;
    }
    break;
  case MODE_LOW_AWAITING_HIGH:
    if ((ticks - cmd->lowSetpoint.timestamp) > cmd->awaitHighLevelTimeout) {
      // Skip the leveling state and go straight to emergency - If we didn't
      // get a high-level command by now, it's probaby never coming.
      cmd->mode = MODE_OFF_EMERGENCY;
    }
    break;
  case MODE_HIGH:
    // TODO: This is a global variable access!! In an ideal design the
    // high-level commander state would be a part of the commander_t.
    if (crtpCommanderHighLevelIsStopped()) {
      // We either just finished landing, or got a high-level emergency stop
      // command, or somehow entered MODE_HIGH erroneously. Either way, it is
      // now time to idle.
      cmd->mode = MODE_OFF_IDLE;
    }
    break;
  }

  // Second pass: Output.
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
    *setpointOut = nullSetpoint;
    break;
  case MODE_LOW:
  case MODE_LOW_AWAITING_HIGH:
    *setpointOut = cmd->lowSetpoint;
    break;
  case MODE_LOW_LEVELING:
    *setpointOut = cmd->lowSetpoint;
    setpointOut->mode.x = modeDisable;
    setpointOut->mode.y = modeDisable;
    setpointOut->mode.roll = modeAbs;
    setpointOut->mode.pitch = modeAbs;
    setpointOut->mode.yaw = modeVelocity;
    setpointOut->attitude.roll = 0;
    setpointOut->attitude.pitch = 0;
    setpointOut->attitudeRate.yaw = 0;
    break;
  case MODE_HIGH:
    crtpCommanderHighLevelGetSetpoint(setpointOut, state);
    break;
  }

  cmd->lastState = *state;
}
