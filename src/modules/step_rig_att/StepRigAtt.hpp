/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <stdint.h>

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

// Rig-only controller that STEP-commands an arbitrary attitude and holds it,
// through the ATTITUDE setpoint path (mc_att_control), for measuring the
// closed-loop step response: rise time, peak time, overshoot, settling time.
//
// This is the deliberate opposite of hold_rig_att, and the difference is the
// entire point of the module. hold_rig_att never publishes the commanded target
// directly: it walks a synthetic "leader" setpoint toward it at HRATT_RATE so
// that mc_att_control only ever sees a small tracking lag, because a large
// static error saturates the rate demand (2*sin(Phi/2)*MC_ROLL_P, clamped at
// MC_ROLLRATE_MAX, saturating past only ~57 degrees of error with stock gains).
// That makes the approach gentle, but it also means the observed trajectory is
// the RAMP's, not the controller's - the ramp rate, not the control law, sets
// how long the move takes.
//
// Here the target is published as a step on the first cycle of OFFBOARD and
// held constant thereafter. Whatever transient follows belongs entirely to
// mc_att_control's attitude-P -> rate-PID cascade and its own saturation
// limits, which is exactly the response a step-response characterization needs
// to measure. The saturation hold_rig_att avoids is not a defect here; it is
// part of the plant being characterized.
//
// Two further differences from hold_rig_att follow from the same purpose:
//
//  1. NO YAW LATCH. hold_rig_att composes its target onto the yaw latched at
//     OFFBOARD entry, so its tilt is relative to wherever the nose happened to
//     point. This module builds q_target = Quatf(Eulerf(roll, pitch, 0))
//     outright - an absolute, always-zero-yaw target. A step-response test
//     wants the same commanded attitude on every run regardless of heading, so
//     runs are comparable to each other.
//
//  2. FIXED THRUST. thrust_body[2] is STEPATT_THRUST verbatim, with no
//     MPC_THR_HOVER fallback and no tilt compensation. The vehicle IS expected
//     to lose altitude while parked at a large tilt - in SITL the test is flown
//     from several hundred metres for exactly that reason. Altitude is spent
//     deliberately to buy a clean attitude measurement.
//
// Never touches position/GPS: a valid attitude estimate is the only
// requirement. It does NOT hold or return to any position. Start it directly
// with `step_rig_att start`; it is not gated by offboard_selector. Large
// commanded angles WILL trip the attitude failure detector (FD_FAIL_R /
// FD_FAIL_P) - see print_usage() and check_failure_detector().
class StepRigAtt : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	StepRigAtt();
	~StepRigAtt() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();

	// Startup/parameter-change sanity checks. Each warns rather than
	// silently clamping: this module's whole job is to command exactly what
	// it was told to, so quietly altering a commanded attitude would
	// invalidate the measurement it exists to produce.
	//
	// The ramp-related checks hold_rig_att carries (rate vs MC_*RATE_MAX,
	// ramp duration, tilt-vs-thrust, Euler degeneracy) are all absent here:
	// there is no ramp rate and no tilt/direction mode for them to check.
	void check_failure_detector() const;
	// COM_DISARM_PRFLT / COM_DISARM_LAND, both of which fire on a vehicle
	// that never reports a takeoff.
	void check_rig_disarm_params() const;
	void check_sibling_modules() const;

	void publish_setpoint(hrt_abstime now, const matrix::Quatf &q_sp);
	void publish_track_current_attitude();
	// The step itself: publish _q_target, unchanged, every cycle.
	void publish_step();

	// q_target = Quatf(Eulerf(STEPATT_ROLL, STEPATT_PITCH, 0)). Absolute and
	// zero-yaw by design - no latched yaw is composed in, unlike
	// hold_rig_att::build_target().
	void build_target();

	// One-shot warning if the step lands inside the post-arm window where an
	// attitude failure trips an undeferrable disarm. Advisory only: the step
	// is never delayed, because delaying it would displace the very transient
	// being measured.
	void warn_if_inside_arm_lockdown(hrt_abstime now) const;

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	uint8_t _nav_state{0};   // NAVIGATION_STATE_MANUAL until the first vehicle_status arrives, i.e. "not OFFBOARD"
	matrix::Quatf _current_att{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};
	hrt_abstime _armed_time{0};

	// No EKF yaw-reset repair here, deliberately, where hold_rig_att needs
	// one. That module carries a latched yaw and a leader quaternion across
	// cycles, so a reset silently staled them. Nothing here survives a cycle
	// in the world frame: _current_att is overwritten wholesale from each
	// vehicle_attitude sample, and _q_target is by definition "zero yaw in
	// whatever frame the estimator is currently using" - which is what a
	// reset redefines, so rotating it would be wrong rather than merely
	// unnecessary.

	bool _was_offboard{false};

	// The commanded attitude. Latched at OFFBOARD entry and then invariant
	// for the whole run - there is no live-retarget option, because a second
	// step mid-run would contaminate the response being measured.
	matrix::Quatf _q_target{1.f, 0.f, 0.f, 0.f};

	// Bookkeeping only - never gates publishing. _step_issued_time is what
	// `step_rig_att status` reports elapsed time against, for correlating the
	// shell view with the log.
	hrt_abstime _step_issued_time{0};

	hrt_abstime _last_publish_timestamp{0};
	uint64_t _setpoints_published{0};
	uint64_t _early_return_disabled{0};
	uint64_t _early_return_no_attitude{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 100};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::STEPATT_EN>) _param_stepatt_en,
		(ParamFloat<px4::params::STEPATT_ROLL>) _param_stepatt_roll,
		(ParamFloat<px4::params::STEPATT_PITCH>) _param_stepatt_pitch,
		(ParamFloat<px4::params::STEPATT_THRUST>) _param_stepatt_thrust,
		(ParamInt<px4::params::STEPATT_PUB_HZ>) _param_stepatt_pub_hz
	)
};
