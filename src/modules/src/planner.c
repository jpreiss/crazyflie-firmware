/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
implementation of planning state machine
*/
#include <stddef.h>
#include "planner.h"

static struct traj_eval plan_eval(struct planner *p, float t);

static void plan_takeoff_or_landing(struct planner *p, float hover_height, float hover_yaw, float duration)
{
	struct vec hover_pos = p->last_known.pos;
	hover_pos.z = hover_height;
	float curr_yaw = p->last_known.yaw;

	piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		p->last_known.pos,  curr_yaw,  vzero(), 0, vzero(),
		hover_pos,          hover_yaw, vzero(), 0, vzero());
}

// ----------------- //
// public functions. //
// ----------------- //

void plan_init(struct planner *p)
{
	p->state = TRAJECTORY_STATE_IDLE;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->reversed = false;
	p->trajectory = NULL;
	p->compressed_trajectory = NULL;
	p->planned_trajectory.pieces = p->pieces;
	p->last_known = traj_eval_zero();
}

void plan_stop(struct planner *p)
{
	p->state = TRAJECTORY_STATE_IDLE;
}

bool plan_is_finished(struct planner *p, float t)
{
	switch (p->type) {
		case TRAJECTORY_TYPE_PIECEWISE:
			return piecewise_is_finished(p->trajectory, t);

		case TRAJECTORY_TYPE_PIECEWISE_COMPRESSED:
		  return piecewise_compressed_is_finished(p->compressed_trajectory, t);

		default:
		  return 1;
	}
}

bool plan_is_stopped(struct planner *p)
{
	return p->state == TRAJECTORY_STATE_IDLE;
}

void plan_tell_last_known_state(struct planner *p, struct vec pos, struct vec vel, float yaw)
{
	p->last_known.pos = pos;
	p->last_known.vel = vel;
	p->last_known.yaw = yaw;
}

struct traj_eval plan_current_goal(struct planner *p, float t)
{
	switch (p->state) {
		case TRAJECTORY_STATE_LANDING:
			if (plan_is_finished(p, t)) {
				p->state = TRAJECTORY_STATE_IDLE;
			}
			// intentional fall-thru
		case TRAJECTORY_STATE_FLYING:
			return plan_eval(p, t);

		default:
			return traj_eval_invalid();
	}
}

struct traj_eval plan_eval(struct planner *p, float t)
{
	switch (p->type) {
		case TRAJECTORY_TYPE_PIECEWISE:
			if (p->reversed) {
				return piecewise_eval_reversed(p->trajectory, t);
			}
			else {
				return piecewise_eval(p->trajectory, t);
			}
			break;

		case TRAJECTORY_TYPE_PIECEWISE_COMPRESSED:
			if (p->reversed) {
				/* not supported */
				return traj_eval_invalid();
			}
			else {
				return piecewise_compressed_eval(p->compressed_trajectory, t);
			}
			break;

		default:
			return traj_eval_invalid();
	}
}

int plan_takeoff(struct planner *p, float hover_height, float hover_yaw, float duration, float t)
{
	if (p->state != TRAJECTORY_STATE_IDLE) {
		return 1;
	}

	plan_takeoff_or_landing(p, hover_height, hover_yaw, duration);
	p->reversed = false;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_land(struct planner *p, float hover_height, float hover_yaw, float duration, float t)
{
	if (p->state == TRAJECTORY_STATE_LANDING) {
		return 1;
	}

	plan_takeoff_or_landing(p, hover_height, hover_yaw, duration);
	p->reversed = false;
	p->state = TRAJECTORY_STATE_LANDING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_go_to(struct planner *p, bool relative, struct vec hover_pos, float hover_yaw, float duration, float t)
{
	if (relative) {
		hover_pos = vadd(hover_pos, p->last_known.pos);
		hover_yaw += p->last_known.yaw;
	}

	piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		p->last_known.pos, p->last_known.yaw, p->last_known.vel, p->last_known.omega.z, p->last_known.acc,
		hover_pos,         hover_yaw,         vzero(),           0,                     vzero());

	p->reversed = false;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_start_trajectory(struct planner *p, struct piecewise_traj* trajectory, bool reversed, bool relative)
{
	p->reversed = reversed;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->trajectory = trajectory;

	if (relative) {
		struct traj_eval traj_init;
		trajectory->shift = vzero();
		if (reversed) {
			traj_init = piecewise_eval_reversed(trajectory, trajectory->t_begin);
		}
		else {
			traj_init = piecewise_eval(trajectory, trajectory->t_begin);
		}
		struct vec shift_pos = vsub(p->last_known.pos, traj_init.pos);
		trajectory->shift = shift_pos;
	}
	else {
		trajectory->shift = vzero();
	}

	return 0;
}

int plan_start_compressed_trajectory( struct planner *p, struct piecewise_traj_compressed* trajectory, bool relative)
{
	p->reversed = 0;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE_COMPRESSED;
	p->compressed_trajectory = trajectory;

	if (relative) {
		trajectory->shift = vzero();
		struct traj_eval traj_init = piecewise_compressed_eval(
			trajectory, trajectory->t_begin
		);
		struct vec shift_pos = vsub(p->last_known.pos, traj_init.pos);
		trajectory->shift = shift_pos;
	} else {
		trajectory->shift = vzero();
	}

	return 0;
}
