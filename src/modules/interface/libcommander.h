#pragma once

#include "planner.h"
#include "stabilizer_types.h"

// This enum has no fixed numerical values on purpose. The values should never
// be relied upon. We want to make sure we can keep the values grouped
// logically in the declaration, instead of in the order they were added.
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

  // Following planner; hover (stay in MODE_HIGH) when trajectory ends.
  MODE_HIGH,

  // Following planner; stop (switch to MODE_OFF_IDLE) when trajectory ends.
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
void libCommanderLowSetpoint(commander_t *cmd, uint32_t millis, setpoint_t const *setpoint);

// Informs the commander that streaming setpoints are about to stop.
// See commander.h comment for more information.
void libCommanderNotifySetpointsStop(commander_t *cmd, uint32_t millis, uint32_t awaitMillis);

// start a takeoff trajectory.
int libCommanderTakeoff(commander_t *p, uint32_t millis, float hover_height, float hover_yaw, float duration);

// start a landing trajectory.
int libCommanderLand(commander_t *p, uint32_t millis, float hover_height, float hover_yaw, float duration);

// move to a given position, then hover there.
int libCommanderGoTo(commander_t *p, uint32_t millis, bool relative, struct vec hover_pos, float hover_yaw, float duration);

// start trajectory. start_from param is ignored if relative == false.
int libCommanderStartTraj(commander_t *p, uint32_t millis, struct piecewise_traj* trajectory, bool reversed, bool relative);

// start compressed trajectory. start_from param is ignored if relative == false.
int libCommanderStartCompressedTraj(commander_t *p, uint32_t millis, struct piecewise_traj_compressed* trajectory, bool relative);

// query if the trajectory is finished.
bool libCommanderTrajIsFinished(commander_t *p, uint32_t millis);

void libCommanderEmergencyStop(commander_t *p);
