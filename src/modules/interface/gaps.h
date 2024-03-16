#pragma once

struct gaps
{
	union {
		struct { float kp_xy; float kp_z; float kd_xy; float kd_z; };
		float theta[4];
	};
	float y[6][4];
};

void gaps_init(float dt);

void gaps_reset(struct gaps *gaps);

void gaps_update(
    float const pos_err[3],
    float const vel_err[3],
    float const p_cost,
    float const v_cost,
    float const u_cost,
    float const eta,
    struct gaps *gaps, // inout
    float u[3] //out
    );
