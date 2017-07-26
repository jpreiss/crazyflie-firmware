#include "math3d.h"

struct tilthex_state {
	struct vec pos;
	struct vec vel;
	struct vec acc;
	struct mat33 R;
	struct vec omega;
};

// output: propeller omega^2
void control(struct tilthex_state s, struct tilthex_state des, float x[6]);
