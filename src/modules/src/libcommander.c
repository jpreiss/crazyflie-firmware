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
  plan_init(&cmd->planner);
}

void libCommanderLowSetpoint(commander_t *cmd, uint32_t millis, setpoint_t const *setpoint)
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

void libCommanderNotifySetpointsStop(commander_t *cmd, uint32_t millis, uint32_t awaitMillis)
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
    // TODO: Should we be doing this constantly in every loop of low modes
    // instead of just once?
    plan_tell_last_known_state(
      &cmd->planner,
      state2vec(cmd->lastState.position),
      state2vec(cmd->lastState.velocity),
      radians(cmd->lastState.attitude.yaw)
    );
    break;
  case MODE_LOW_LEVELING:
    // TODO: What should happen?
    break;
  case MODE_LOW_AWAITING_HIGH:
    // TODO: How to reconcile old and new await timers?
    break;
  }
}

void libCommanderHighLevelRecvd(commander_t *cmd, uint32_t millis, bool landing)
{
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_LOW_AWAITING_HIGH:
  case MODE_HIGH:
    cmd->mode = landing ? MODE_HIGH_LANDING : MODE_HIGH;
  case MODE_OFF_EMERGENCY:
  case MODE_LOW:
  case MODE_LOW_LEVELING:
    // Do nothing - overridden!
    break;
  }
}

void libCommanderStep(commander_t *cmd, uint32_t millis, state_t const *state, setpoint_t *setpointOut)
{
  // First pass: Mode changes due to passage of time.
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
  case MODE_HIGH:
    // No time-triggered state change possible.
    break;
  case MODE_LOW:
    if ((millis - cmd->lowSetpoint.timestamp) > cmd->levelingTimeout) {
      cmd->mode = MODE_LOW_LEVELING;
    }
    // Technically we could fall through to MODE_LOW_LEVELING in case we passed
    // both timeouts in the same step, but this should never happen with
    // reasonable timeout values and stabilizer loop rate.
    break;
  case MODE_LOW_LEVELING:
    if ((millis - cmd->lowSetpoint.timestamp) > cmd->emergencyTimeout) {
      cmd->mode = MODE_OFF_EMERGENCY;
    }
    break;
  case MODE_LOW_AWAITING_HIGH:
    if ((millis - cmd->lowSetpoint.timestamp) > cmd->awaitHighLevelTimeout) {
      // Skip the leveling state and go straight to emergency - If we didn't
      // get a high-level command by now, it's probaby never coming.
      cmd->mode = MODE_OFF_EMERGENCY;
    }
    break;
  case MODE_HIGH_LANDING:
    if (plan_is_finished(&cmd->planner)) {
      cmd->mode = MODE_OFF_IDLE;
    }
    break;
  }

  struct traj_eval trajEval;
  // TODO: It's ugly that the high-level planner uses its own timestamp.
  float const t = usecTimestamp() / 1e6;

  // Second pass: Computation, with possible state change due to errors.
  switch (state->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
  case MODE_LOW:
  case MODE_LOW_LEVELING:
  case MODE_LOW_AWAITING_HIGH:
    break;
  case MODE_HIGH:
  case MODE_HIGH_LANDING:
    trajEval = plan_current_goal(&cmd->planner, t);
    if (!is_traj_eval_valid(&trajEval)) {
      // Programming error!
      cmd->mode = MODE_OFF_EMERGENCY;
    }
    break;
  default:
    // TODO: error!
    break;
  }

  // Third pass: Output.
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
    *setpointOut = nullSetpoint;
    // If we are on the ground, update the setpoint with the current state so
    // we can take off correctly. TODO: Is it OK to do this in emergency? Or
    // should be idle only?
    plan_tell_last_known_state(
      &cmd->planner,
      state2vec(state->position),
      state2vec(state->velocity),
      radians(state->attitude.yaw)
    );
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
  case MODE_HIGH_LANDING:
    setpoint->position.x = ev.pos.x;
    setpoint->position.y = ev.pos.y;
    setpoint->position.z = ev.pos.z;
    setpoint->velocity.x = ev.vel.x;
    setpoint->velocity.y = ev.vel.y;
    setpoint->velocity.z = ev.vel.z;
    setpoint->attitude.yaw = degrees(ev.yaw);
    setpoint->attitudeRate.roll = degrees(ev.omega.x);
    setpoint->attitudeRate.pitch = degrees(ev.omega.y);
    setpoint->attitudeRate.yaw = degrees(ev.omega.z);
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->mode.quat = modeDisable;
    setpoint->acceleration.x = ev.acc.x;
    setpoint->acceleration.y = ev.acc.y;
    setpoint->acceleration.z = ev.acc.z;

    // Store *setpoint*, not state, in planner. This is important when
    // executing a sequence of relative goTos that return to the starting
    // position in the end. We should not accumulate error.
    plan_tell_last_known_state(&planner, trajEval.pos, trajEval.vel, trajEval.yaw);
    break;
  }

  cmd->lastState = *state;
}
