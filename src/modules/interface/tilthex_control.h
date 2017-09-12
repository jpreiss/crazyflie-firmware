#include "math3d.h"

struct tilthex_state {
	struct vec pos;
	struct vec vel;
	struct vec acc;
	struct mat33 R;
	struct vec omega;
	struct vec pos_err_integral;
};

// output: propeller omega^2
// NOTE: the controller keeps its integral terms in the state struct,
// so the argument "state" must be persisted between calls
void tilthex_control(struct tilthex_state *state, struct tilthex_state const des, float dt, float x[6]);
