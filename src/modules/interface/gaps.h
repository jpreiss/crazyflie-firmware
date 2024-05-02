#pragma once


struct gaps
{
	union {
		struct {
			float kp_xy; float kp_z;
			float kd_xy; float kd_z;
			float ki_xy; float ki_z;
		};
		float theta[6];
	};
	// adadelta state
	float grad_accum[6];
	float update_accum[6];
	// state is position error, velocity error, integrated position error
	float y[9][6];
	// useful for logging/debugging - max abs value of y
	float yabsmax;
};

void gaps_init(float dt);

void gaps_reset(struct gaps *gaps);

enum gaps_opt {
	GAPS3DOF_OPT_OGD = 0,
	GAPS3DOF_OPT_ADADELTA,
};

void gaps_update(
    float const pos_err[3],
    float const vel_err[3],
    float const int_pos_err[3],
    float const p_cost,
    float const v_cost,
    float const u_cost,
    float const eta,
    float const damping,
    float const ad_decay,
    float const ad_eps,
    enum gaps_opt opt,
    struct gaps *gaps, // inout
    float u[3] //out
    );
