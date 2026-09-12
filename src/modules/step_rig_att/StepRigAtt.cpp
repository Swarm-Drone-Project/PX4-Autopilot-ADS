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

#include "StepRigAtt.hpp"

#include <inttypes.h>
#include <math.h>

#include <mathlib/math/Limits.hpp>
#include <parameters/param.h>

using matrix::Eulerf;
using matrix::Quatf;

using step_rig_att::decompose;
using step_rig_att::target_euler;
using step_rig_att::target_tilt_direction;
using step_rig_att::TargetMode;

ModuleBase::Descriptor StepRigAtt::desc{task_spawn, custom_command, print_usage};

StepRigAtt::StepRigAtt() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool StepRigAtt::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

Quatf StepRigAtt::target_from_params() const
{
	if (static_cast<TargetMode>(_param_stepatt_mode.get()) == TargetMode::EulerRollPitch) {
		return target_euler(_param_stepatt_roll.get(), _param_stepatt_pitch.get());
	}

	// Tilt + direction: a rotation about a HORIZONTAL axis, so it asks for no
	// yaw rotation at all and the controller can deliver every degree of it on
	// the two strong axes. See STEPATT_DIR.
	return target_tilt_direction(_param_stepatt_tilt.get(), _param_stepatt_dir.get());
}

void StepRigAtt::build_target()
{
	// The target is absolute and zero-yaw: no latched heading is composed in,
	// so repeating a run with the vehicle pointing a different way commands
	// the same attitude and yields a comparable measurement.
	_q_target = target_from_params();
	_q_target.normalize();
}

void StepRigAtt::warn_if_inside_arm_lockdown(hrt_abstime now) const
{
	if (_armed_time == 0) {
		// Not armed, or no vehicle_status yet. The undeferrable-disarm
		// window is measured from arming, so there is nothing to warn about.
		return;
	}

	float lockdown_s = 3.f;
	float spoolup_s = 0.f;
	param_t p = param_find("COM_LKDOWN_TKO");

	if (p != PARAM_INVALID) {
		param_get(p, &lockdown_s);
	}

	p = param_find("COM_SPOOLUP_TIME");

	if (p != PARAM_INVALID) {
		param_get(p, &spoolup_s);
	}

	const float armed_for_s = static_cast<float>(now - _armed_time) * 1e-6f;

	if (armed_for_s > (lockdown_s + spoolup_s)) {
		return;
	}

	// hold_rig_att defers its ramp past this window (HRATT_ARM_GATE). This
	// module cannot: delaying the step would move the transient it exists to
	// measure. So it says so instead, once, at the moment it matters.
	PX4_ERR("Step issued %.1f s after arming, inside COM_LKDOWN_TKO+COM_SPOOLUP_TIME.", (double)armed_for_s);
	PX4_ERR("An attitude failure here DISARMS undeferrably. Wait out that window before OFFBOARD.");
}

void StepRigAtt::parameters_updated()
{
	updateParams();

	// Clamped to the declared range rather than trusted: below 25 Hz the
	// commander's OFFBOARD heartbeat lapses and the mode drops out mid-test,
	// which looks like a control failure in the log but is not one.
	const int requested_hz = _param_stepatt_pub_hz.get();
	const int clamped_hz = math::constrain(requested_hz, 25, 250);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}

	check_euler_degeneracy();
	check_failure_detector();
	check_rig_disarm_params();
	check_sibling_modules();
}

void StepRigAtt::check_euler_degeneracy() const
{
	if (static_cast<TargetMode>(_param_stepatt_mode.get()) != TargetMode::EulerRollPitch) {
		return;
	}

	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(target_euler(_param_stepatt_roll.get(), _param_stepatt_pitch.get()),
		  tilt_deg, dir_deg, dyaw_deg);

	// Printed for every mode-1 command, degenerate or not: it is the one line
	// that says what the vehicle will actually do, and it hands over the
	// mode-0 parameters that reproduce the same lean.
	PX4_INFO("MODE 1 roll %.0f pitch %.0f == tilt %.0f deg, dir %.0f deg, plus %.0f deg delta-yaw.",
		 (double)_param_stepatt_roll.get(), (double)_param_stepatt_pitch.get(),
		 (double)tilt_deg, (double)dir_deg, (double)dyaw_deg);

	if (fabsf(dyaw_deg) <= 15.f) {
		return;
	}

	// Past this the command is no longer the attitude it looks like. The
	// delta-yaw share is not discarded - AttitudeControl::update() splits it
	// out and scales it by MC_YAW_WEIGHT (0.4 by default) because yaw is the
	// weakest axis a multicopter has - so on the vehicle it reads as "the roll
	// arrived and the pitch never did".
	PX4_ERR("%.0f deg of this command is delta-yaw, run at MC_YAW_WEIGHT on the weakest axis.",
		(double)fabsf(dyaw_deg));
	PX4_ERR("It will not arrive. Use STEPATT_MODE 0 with STEPATT_TILT %.0f, STEPATT_DIR %.0f.",
		(double)tilt_deg, (double)dir_deg);
}

void StepRigAtt::check_failure_detector() const
{
	// FailureDetector::updateAttitudeStatus() compares EULER roll and pitch
	// against FD_FAIL_R/FD_FAIL_P, so those are the quantities to check - not
	// the tilt magnitude, which is a different number. Taken from the built
	// target rather than the parameters, because in tilt+direction mode the
	// commanded numbers are not Euler angles at all: tilt 45 / dir 45 is Euler
	// roll 35.3 / pitch -30.
	//
	// The check is gated on flag_control_attitude_enabled, which attitude-mode
	// OFFBOARD sets, so it is armed for the entire run.
	const Eulerf euler(target_from_params());
	const float roll_deg = fabsf(math::degrees(euler.phi()));
	const float pitch_deg = fabsf(math::degrees(euler.theta()));

	float fail_r = 0.f;
	float fail_p = 0.f;
	int32_t fail_r_i = 0;
	int32_t fail_p_i = 0;
	param_t p = param_find("FD_FAIL_R");

	if ((p != PARAM_INVALID) && (param_get(p, &fail_r_i) == PX4_OK)) {
		fail_r = (float)fail_r_i;
	}

	p = param_find("FD_FAIL_P");

	if ((p != PARAM_INVALID) && (param_get(p, &fail_p_i) == PX4_OK)) {
		fail_p = (float)fail_p_i;
	}

	const bool trips_roll = (fail_r > 0.f) && (roll_deg > fail_r);
	const bool trips_pitch = (fail_p > 0.f) && (pitch_deg > fail_p);

	if (!trips_roll && !trips_pitch) {
		return;
	}

	// Split across several lines rather than appended: log_message_s.text
	// caps at 127 chars and the tail - the actionable part - would be chopped.
	PX4_ERR("Target roll %.0f/pitch %.0f exceeds FD_FAIL_R %.0f/FD_FAIL_P %.0f - set both to 0.",
		(double)roll_deg, (double)pitch_deg, (double)fail_r, (double)fail_p);

	// The reason this matters more here than on a ramped module: a step
	// crosses the threshold on the first cycle, so the 0.3 s trigger expires
	// while the response is still in its rise - the trip lands in the middle
	// of the measurement rather than after it.
	PX4_ERR("A step crosses that threshold immediately: the trip lands mid-rise and voids the run.");

	int32_t cbrk = 0;
	p = param_find("CBRK_FLIGHTTERM");

	if ((p != PARAM_INVALID) && (param_get(p, &cbrk) == PX4_OK) && (cbrk != 121212)) {
		PX4_ERR("CBRK_FLIGHTTERM is %" PRId32 ", not 121212: the trip will TERMINATE flight, not warn.",
			cbrk);
	}
}

void StepRigAtt::check_rig_disarm_params() const
{
	// With a fixed thrust setpoint and no altitude loop, the land detector's
	// low-thrust branch can latch "landed" outright, and a vehicle that never
	// reports a takeoff trips the preflight disarm timer. Both end the run
	// with an auto-disarm that has nothing to do with attitude.
	float disarm_prflt = 0.f;
	param_t p = param_find("COM_DISARM_PRFLT");

	if ((p != PARAM_INVALID) && (param_get(p, &disarm_prflt) == PX4_OK) && (disarm_prflt > 0.f)) {
		PX4_WARN("COM_DISARM_PRFLT is %.0f s: no takeoff is detected here, so it disarms. Set 0.",
			 (double)disarm_prflt);
	}

	float disarm_land = 0.f;
	p = param_find("COM_DISARM_LAND");

	if ((p != PARAM_INVALID) && (param_get(p, &disarm_land) == PX4_OK) && (disarm_land > 0.f)) {
		PX4_WARN("COM_DISARM_LAND is %.0f s: the land detector can latch here and disarm. Set 0.",
			 (double)disarm_land);
	}

	int32_t airmode = 0;
	p = param_find("MC_AIRMODE");

	if ((p != PARAM_INVALID) && (param_get(p, &airmode) == PX4_OK) && (airmode == 0)) {
		// Worth knowing before reading a step response: the collective-thrust
		// dip during the saturated part of the transient is the allocator
		// paying for roll/pitch torque, not a thrust command from this module.
		PX4_INFO("MC_AIRMODE is 0: saturating roll/pitch torque is paid for out of collective thrust.");
	}
}

void StepRigAtt::check_sibling_modules() const
{
	// mc_att_control performs no publisher-identity check: it takes whichever
	// vehicle_attitude_setpoint carries the newest timestamp. Two modules
	// publishing it therefore produce no error anywhere - they interleave at
	// loop rate and the vehicle chases the average, which on a step-response
	// measurement reads as an inexplicably sluggish or noisy rise.
	static const char *const kSiblingEnables[] = {"HRATT_EN", "FRIGAX_EN", "FRIGPA_EN", "FRIG_EN", "FRIGP_EN"};

	for (const char *name : kSiblingEnables) {
		param_t p = param_find(name);

		if (p == PARAM_INVALID) {
			continue;
		}

		int32_t enabled = 0;

		if ((param_get(p, &enabled) == PX4_OK) && (enabled != 0)) {
			PX4_ERR("%s is enabled and publishes the same topics - setpoints will interleave. Disable it.",
				name);
		}
	}
}

void StepRigAtt::publish_setpoint(hrt_abstime now, const Quatf &q_sp)
{
	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = false;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = true;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_control_mode_pub.publish(ocm);

	vehicle_attitude_setpoint_s sp{};
	// The timestamp is load-bearing, not decoration: mc_att_control rejects
	// any setpoint that is not strictly newer than the last one it accepted.
	sp.timestamp = now;
	q_sp.copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	// Verbatim, with no tilt compensation and no MPC_THR_HOVER fallback: the
	// altitude lost while parked at a large tilt is the price of the
	// measurement, not a fault to be corrected. See STEPATT_THRUST.
	sp.thrust_body[2] = _param_stepatt_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void StepRigAtt::publish_track_current_attitude()
{
	// Pre-OFFBOARD: mirror the vehicle's live attitude every cycle, so
	// commander's pre-switch check ("is a signal already being published?")
	// passes and there is no step until the operator asks for one. Only
	// vehicle_attitude is used, so this admits the OFFBOARD switch with no
	// GPS/position estimate at all.
	publish_setpoint(hrt_absolute_time(), _current_att);
}

void StepRigAtt::publish_step()
{
	// The same quaternion, every cycle, for as long as OFFBOARD is engaged.
	// Nothing shapes, filters or advances it - that absence IS the feature:
	// every millisecond of the response that follows is mc_att_control's.
	publish_setpoint(hrt_absolute_time(), _q_target);
}

void StepRigAtt::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		parameters_updated();
	}

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
		_armed_time = status.armed_time;
	}

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_current_att = Quatf(att.q);
		_has_attitude = true;
	}

	if (!_param_stepatt_en.get()) {
		_was_offboard = false;
		++_early_return_disabled;
		return;
	}

	if (!_has_attitude) {
		// Nothing sane to publish yet - wait for the first attitude sample.
		// IMU/mag convergence only, not GPS: clears in well under a second.
		++_early_return_no_attitude;
		return;
	}

	if (_nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		// Leaving OFFBOARD re-arms the module for a fresh step, so a run can
		// be repeated by cycling the mode without restarting anything.
		_was_offboard = false;
		publish_track_current_attitude();
		return;
	}

	if (!_was_offboard) {
		// Rising edge into OFFBOARD. This cycle is the step: build the target
		// and publish it immediately, with nothing in between. No arm gate
		// and no ramp stand between the mode switch and the command.
		const hrt_abstime now = hrt_absolute_time();

		build_target();
		_step_issued_time = now;
		_was_offboard = true;

		warn_if_inside_arm_lockdown(now);

		// Report the step as tilt and direction whichever mode built it: that
		// is the pair the vehicle will actually fly, and in euler mode it is
		// the number that differs from what was typed.
		float tilt_deg = 0.f;
		float dir_deg = 0.f;
		float dyaw_deg = 0.f;
		decompose(_q_target, tilt_deg, dir_deg, dyaw_deg);

		PX4_INFO("STEP: tilt %.1f deg, dir %.1f deg, thrust %.2f - response is now mc_att_control's.",
			 (double)tilt_deg, (double)dir_deg, (double)_param_stepatt_thrust.get());
	}

	publish_step();
}

int StepRigAtt::task_spawn(int argc, char *argv[])
{
	StepRigAtt *instance = new StepRigAtt();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int StepRigAtt::print_status()
{
	const bool euler_mode = (static_cast<TargetMode>(_param_stepatt_mode.get()) == TargetMode::EulerRollPitch);

	if (euler_mode) {
		PX4_INFO("enabled=%d mode=euler roll=%.1f pitch=%.1f thrust=%.2f pub_hz=%" PRId32,
			 (int)_param_stepatt_en.get(),
			 (double)_param_stepatt_roll.get(),
			 (double)_param_stepatt_pitch.get(),
			 (double)_param_stepatt_thrust.get(),
			 (int32_t)_param_stepatt_pub_hz.get());

	} else {
		PX4_INFO("enabled=%d mode=tilt+dir tilt=%.1f dir=%.1f thrust=%.2f pub_hz=%" PRId32,
			 (int)_param_stepatt_en.get(),
			 (double)_param_stepatt_tilt.get(),
			 (double)_param_stepatt_dir.get(),
			 (double)_param_stepatt_thrust.get(),
			 (int32_t)_param_stepatt_pub_hz.get());
	}

	// The decomposition rather than an Euler readback: Euler extraction is not
	// injective near pitch = +-90, so it prints a different triple for the same
	// orientation, and it cannot show the delta-yaw share at all - which is the
	// one number that says whether the command will actually arrive.
	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(target_from_params(), tilt_deg, dir_deg, dyaw_deg);

	PX4_INFO("target: tilt=%.1f deg dir=%.1f deg delta_yaw=%.1f deg",
		 (double)tilt_deg, (double)dir_deg, (double)dyaw_deg);

	PX4_INFO("q_target=[%.4f %.4f %.4f %.4f] (yaw 0, absolute)",
		 (double)_q_target(0), (double)_q_target(1), (double)_q_target(2), (double)_q_target(3));

	if (_was_offboard) {
		const float since_step_s = static_cast<float>(hrt_absolute_time() - _step_issued_time) * 1e-6f;
		PX4_INFO("state=STEP issued %.2f s ago", (double)since_step_s);

	} else {
		PX4_INFO("state=TRACK (mirroring live attitude, waiting for OFFBOARD)");
	}

	PX4_INFO("tx=%" PRIu64 " early_disabled=%" PRIu64 " early_no_attitude=%" PRIu64,
		 _setpoints_published, _early_return_disabled, _early_return_no_attitude);

	return 0;
}

int StepRigAtt::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int StepRigAtt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`step_rig_att` publishes a commanded attitude as a STEP and holds it, so that
the closed-loop step response of the attitude controller - rise time, peak time,
overshoot, settling time - can be measured directly. It is a test instrument,
not a flight mode.

Start it with `step_rig_att start`. The instant OFFBOARD is engaged it builds
the target from STEPATT_MODE and publishes that same quaternion every cycle for
as long as OFFBOARD stays engaged, with STEPATT_THRUST passed straight through
as thrust_body[2]. Leaving OFFBOARD re-arms it for a fresh step. Before OFFBOARD
it mirrors the live attitude, which is what lets commander admit the mode switch
at all. It never touches position or GPS.

#### Say where to lean, not how to roll

STEPATT_MODE 0 (the default) takes a lean angle and a direction:

    STEPATT_TILT  how far to lean from vertical
    STEPATT_DIR   which way, 0 = toward the nose/north, 90 = right wing/east

so "lean 45 degrees toward the north-east" is TILT 45, DIR 45. Yaw is always
zero, so the nose points north and STEPATT_DIR reads as a compass bearing.

STEPATT_MODE 1 takes Euler roll and pitch instead, and exists because
single-axis steps are the cleanest thing to measure a rise time from. Use it for
roll-only or pitch-only tests. Do NOT use it for a two-axis lean: in intrinsic
Z-Y-X the pitch is applied about the already-rolled axis, so the resulting
orientation contains a rotation about body z. AttitudeControl::update() splits
that out as delta-yaw and scales it by MC_YAW_WEIGHT (0.4) because yaw is the
weakest axis a multicopter has - it comes only from differential rotor drag. A
commanded roll 65 / pitch 60 is really

    tilt 77.8 deg  +  40.4 deg of delta-yaw

and those 40 degrees never arrive: on the vehicle it reads as "the roll arrived
and the pitch never did". Mode 0 has exactly zero delta-yaw at any tilt in any
direction, so all of it is delivered on the two strong axes. The startup
diagnostic prints the mode-0 equivalent of any mode-1 command, including its
delta-yaw share.

Judge the result by tilt, not by an Euler readback: a correct TILT 45 / DIR 45
hold reads back as roll 35.3, pitch -30.0, yaw -9.7 - the same orientation, a
different triple, because Euler extraction is not injective.
`step_rig_att status` reports tilt, direction and delta-yaw for that reason.

#### Why this is the opposite of hold_rig_att

`hold_rig_att` exists precisely to avoid stepping. It walks an internal "leader"
setpoint to the target at HRATT_RATE, because `AttitudeControl::update()` turns
attitude error into a rate demand of `2*sin(Phi/2)*MC_ROLL_P`, hard-clamped at
MC_ROLLRATE_MAX - with stock gains (4.0, 220 deg/s) that clamp is already
reached at Phi = 57 degrees, so a large static command slams in at full
authority.

That is the correct design for moving a rig gently, and the wrong one for
measuring a controller, because the trajectory you then observe is the ramp's:
HRATT_RATE sets how long the move takes, not the control law. This module
removes the ramp entirely. The saturation `hold_rig_att` avoids is not a defect
to be engineered around here - it is part of the plant under test, and a rise
time measured through it is the real one.

Consequently there is no rate parameter, no ramp timeout, no live-retarget
option (a second step mid-run would contaminate the response), and no arm gate
(delaying the step would displace the transient being measured).

#### The target is absolute and zero-yaw

`hold_rig_att` composes its target onto the yaw latched at OFFBOARD entry, so
its tilt is relative to where the nose points. This module does not: the target
is `Eulerf(roll, pitch, 0)` outright. Repeating a run with the vehicle facing a
different direction therefore commands the same attitude, which is what makes
successive runs comparable.

Note that Euler roll/pitch remains a lossy parameterization at large angles -
in intrinsic Z-Y-X, pitch is applied about the already-rolled axis, so at roll
90 degrees the pitch command stops changing tilt and is routed into the
delta-yaw channel at MC_YAW_WEIGHT instead. That is not a bug to be fixed here;
it is a property of commanding roll and pitch, and it will show up in the
measured response as a pitch axis that never arrives. Use `hold_rig_att`'s
HRATT_MODE 0 (tilt + direction) if a non-degenerate parameterization is wanted.

#### Failsafes that will fire

The attitude failure detector compares Euler roll and pitch against FD_FAIL_R /
FD_FAIL_P (both 60 degrees by default, 0.3 s trigger) whenever attitude control
is enabled - which is exactly this module's operating condition. A step crosses
the threshold on its first cycle, so the trip lands about 0.3 s later, in the
middle of the rise: the run is voided rather than merely warned about. Set
FD_FAIL_R and FD_FAIL_P to 0 before testing past 60 degrees.

Worse, within COM_LKDOWN_TKO + COM_SPOOLUP_TIME of arming the trip action is an
unconditional DISARM marked `cannotBeDeferred()` - no circuit breaker or
deferral suppresses it. Arm, wait out that window, and only then switch to
OFFBOARD; the module logs an error if the step lands inside it.

COM_DISARM_PRFLT and COM_DISARM_LAND also fire here, since no takeoff is ever
detected and the land detector's low-thrust branch can latch. All of these are
checked at startup and on every parameter change, and warned about rather than
silently corrected - correcting them would alter the command being measured.

#### Altitude is spent deliberately

Thrust is fixed at STEPATT_THRUST with no compensation, so lift authority falls
with cos(tilt) and reverses past 90 degrees. The vehicle IS expected to descend
while holding a large commanded attitude. In SITL, start the test several
hundred metres up so the transient completes long before ground contact:

    export PX4_GZ_MODEL_POSE="0,0,750,0,0,0"
    make px4_sitl gz_x500

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("step_rig_att", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int step_rig_att_main(int argc, char *argv[])
{
	return ModuleBase::main(StepRigAtt::desc, argc, argv);
}
