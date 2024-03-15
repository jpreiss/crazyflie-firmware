#pragma once

void gaps_init(
    float kp_xy, float kp_z, float kd_xy, float kd_z, float dt);

void gaps_update(
    float const pos_err[3],
    float const vel_err[3],
    float const p_cost,
    float const v_cost,
    float const u_cost,
    float const eta,
    float u[3] //out
    );
