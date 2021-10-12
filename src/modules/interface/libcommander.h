#pragma once

#include "planner.h"
#include "stabilizer_types.h"

// This enum intentionally has no fixed numerical values.
enum cmdMode {

  // Motors off, in a rest state, sitting on ground/platform.
  MODE_OFF_IDLE,

  // Motors off because something bad happened.
  MODE_OFF_EMERGENCY,

  // Following low-level streaming setpoints.
  MODE_LOW,

  // TODO: Implement priority system?

  // If we go too long in MODE_LOW without receiving a setpoint, disable all
  // control in the x/y plane and try to hold zero attitude at the current
  // altitude (or z velocity if that was the previous control mode.)
  MODE_LOW_LEVELING,

  // Holding the last low-level streaming setpoint, but switch to high-level
  // as soon as a high-level command is received. This is a separate mode
  // because normally the low-level commands preempt the high-level commands
  // to allow for manual takeover from a failing automated system.
  MODE_LOW_AWAITING_HIGH,

  // Following planner. When trajectory ends, stay in MODE_HIGH to hover.
  MODE_HIGH,

  // Following planner. When trajectory ends, switch to MODE_OFF_IDLE to cut
  // motors.
  MODE_HIGH_LANDING,
};

typedef struct commander_s
{
  // STATE
  enum cmdMode mode;
  setpoint_t lowSetpoint;
  state_t lastState;
  uint32_t awaitHighLevelTimeout;
  struct planner planner;
  struct traj_eval highStartFrom;

  // CONSTANTS / PARAMS
  uint32_t levelingTimeout;
  uint32_t emergencyTimeout;
} commander_t;

// Initializes commander state (to MODE_OFF_IDLE).
void libCommanderInit(commander_t *cmd, uint32_t levelingTimeout, uint32_t emergencyTimeout);

// Applies any state change required by the passage of time, then fills the
// output setpoint.
void libCommanderStep(commander_t *cmd, uint32_t millis, state_t const *state, setpoint_t *setpointOut);

// Processes a low-level setpoint. Preempts high-level mode!
int libCommanderLowSetpoint(commander_t *cmd, uint32_t millis, setpoint_t const *setpoint);

// Inform the commander that low-level setpoints are about to stop. The
// commander will hold the last low-level setpoint for awaitMillis. If a
// high-level command has still not arrived after awaitMillis, declare an
// emergency.
int libCommanderNotifySetpointsStop(commander_t *cmd, uint32_t millis, uint32_t awaitMillis);

// Start a takeoff trajectory.
int libCommanderTakeoff(commander_t *p, uint32_t millis, float hover_height, float hover_yaw, float duration);

// Start a landing trajectory. Motor power will be cut afterwards.
int libCommanderLand(commander_t *p, uint32_t millis, float hover_height, float hover_yaw, float duration);

// Move to a given position, then hover there.
int libCommanderGoTo(commander_t *p, uint32_t millis, bool relative, struct vec hover_pos, float hover_yaw, float duration);

// Start a trajectory. The start_from argument is ignored if
// relative == false.
int libCommanderStartTraj(commander_t *p, uint32_t millis, struct piecewise_traj* trajectory, bool reversed, bool relative);

// Start a compressed trajectory. The start_from argument is ignored if
// relative == false.
int libCommanderStartCompressedTraj(commander_t *p, uint32_t millis, struct piecewise_traj_compressed* trajectory, bool relative);

// Query if the trajectory is finished.
int libCommanderTrajIsFinished(commander_t *p, uint32_t millis, bool *output);

// Cut motor power. This command cannot fail.
void libCommanderEmergencyStop(commander_t *p);
