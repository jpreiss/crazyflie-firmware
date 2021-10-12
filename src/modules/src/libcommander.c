#include "libcommander.h"

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

static struct vec state2vec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}


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
  case MODE_HIGH_LANDING:
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
  case MODE_HIGH_LANDING:
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
    if (plan_is_finished(&cmd->planner, millis / 1e3)) {
      cmd->mode = MODE_OFF_IDLE;
    }
    break;
  }

  struct traj_eval trajEval;

  // Second pass: Computation, with possible state change due to errors.
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_OFF_EMERGENCY:
  case MODE_LOW:
  case MODE_LOW_LEVELING:
  case MODE_LOW_AWAITING_HIGH:
    break;
  case MODE_HIGH:
  case MODE_HIGH_LANDING:
    trajEval = plan_eval(&cmd->planner, millis / 1e3);
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
    setpointOut->position.x = trajEval.pos.x;
    setpointOut->position.y = trajEval.pos.y;
    setpointOut->position.z = trajEval.pos.z;
    setpointOut->velocity.x = trajEval.vel.x;
    setpointOut->velocity.y = trajEval.vel.y;
    setpointOut->velocity.z = trajEval.vel.z;
    setpointOut->attitude.yaw = degrees(trajEval.yaw);
    setpointOut->attitudeRate.roll = degrees(trajEval.omega.x);
    setpointOut->attitudeRate.pitch = degrees(trajEval.omega.y);
    setpointOut->attitudeRate.yaw = degrees(trajEval.omega.z);
    setpointOut->mode.x = modeAbs;
    setpointOut->mode.y = modeAbs;
    setpointOut->mode.z = modeAbs;
    setpointOut->mode.roll = modeDisable;
    setpointOut->mode.pitch = modeDisable;
    setpointOut->mode.yaw = modeAbs;
    setpointOut->mode.quat = modeDisable;
    setpointOut->acceleration.x = trajEval.acc.x;
    setpointOut->acceleration.y = trajEval.acc.y;
    setpointOut->acceleration.z = trajEval.acc.z;

    // Store *setpoint*, not state, in planner. This is important when
    // executing a sequence of relative goTos that return to the starting
    // position in the end. We should not accumulate error.
    plan_tell_last_known_state(&cmd->planner, trajEval.pos, trajEval.vel, trajEval.yaw);
    break;
  }

  cmd->lastState = *state;
}

// start a takeoff trajectory.
int libCommanderTakeoff(commander_t *cmd, uint32_t millis, float hover_height, float hover_yaw, float duration)
{
  float t = millis / 1000;
  int result = 0;
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_HIGH:
    result = plan_takeoff(&cmd->planner, hover_height, hover_yaw, duration, t);
    if (result == 0) {
      cmd->mode = MODE_HIGH;
    }
    return result;
  default:
    // TODO: Signal error?
    return 0;
  }
}

// start a landing trajectory.
int libCommanderLand(commander_t *cmd, uint32_t millis, float hover_height, float hover_yaw, float duration)
{
  float t = millis / 1000;
  int result = 0;
  switch (cmd->mode) {
  case MODE_HIGH:
  case MODE_LOW_AWAITING_HIGH:
    result = plan_land(&cmd->planner, hover_height, hover_yaw, duration, t);
    if (result == 0) {
      cmd->mode = MODE_HIGH_LANDING;
    }
    return result;
  default:
    // TODO: Signal error?
    return 0;
  }
}

// move to a given position, then hover there.
int libCommanderGoTo(commander_t *cmd, uint32_t millis, bool relative, struct vec hover_pos, float hover_yaw, float duration)
{
  float t = millis / 1000;
  int result = 0;
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_HIGH:
  case MODE_LOW_AWAITING_HIGH:
    result = plan_go_to(&cmd->planner, relative, hover_pos, hover_yaw, duration, t);
    if (result == 0) {
      cmd->mode = MODE_HIGH;
    }
    return result;
  default:
    // TODO: Signal error?
    return 0;
  }
}

// start trajectory. start_from param is ignored if relative == false.
int libCommanderStartTraj(commander_t *cmd, uint32_t millis, struct piecewise_traj* trajectory, bool reversed, bool relative)
{
  float t = millis / 1000;
  int result = 0;
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_HIGH:
  case MODE_LOW_AWAITING_HIGH:
    trajectory->t_begin = t;
    result = plan_start_trajectory(&cmd->planner, trajectory, reversed, relative);
    if (result == 0) {
      cmd->mode = MODE_HIGH;
    }
    return result;
  default:
    // TODO: Signal error?
    return 0;
  }
}

// start compressed trajectory. start_from param is ignored if relative == false.
int libCommanderStartCompressedTraj(commander_t *cmd, uint32_t millis, struct piecewise_traj_compressed* trajectory, bool relative)
{
  float t = millis / 1000;
  int result = 0;
  switch (cmd->mode) {
  case MODE_OFF_IDLE:
  case MODE_HIGH:
  case MODE_LOW_AWAITING_HIGH:
    trajectory->t_begin = t;
    result = plan_start_compressed_trajectory(&cmd->planner, trajectory, relative);
    if (result == 0) {
      cmd->mode = MODE_HIGH;
    }
    return result;
  default:
    // TODO: Signal error?
    return 0;
  }
}

// query if the trajectory is finished.
bool libCommanderTrajIsFinished(commander_t *cmd, uint32_t millis)
{
  float t = millis / 1000;
  switch (cmd->mode) {
  case MODE_HIGH:
  case MODE_HIGH_LANDING:
    return plan_is_finished(&cmd->planner, t);
  default:
    // TODO: Signal error?
    return 0;
  }
}

void libCommanderEmergencyStop(commander_t *cmd)
{
  cmd->mode = MODE_OFF_EMERGENCY;
}
