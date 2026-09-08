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

#include "MotorRigTest.hpp"

#include <inttypes.h>
#include <string.h>

#include <mathlib/math/Limits.hpp>

// Same entry point the `logger on` / `logger off` shell commands invoke;
// linked into the same firmware image, so calling it directly here reaches
// Logger::set_arm_override() with no new logging mechanism required.
extern "C" __EXPORT int logger_main(int argc, char *argv[]);

ModuleBase::Descriptor MotorRigTest::desc{task_spawn, custom_command, print_usage};

MotorRigTest::MotorRigTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool MotorRigTest::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

void MotorRigTest::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_mrigt_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}
}

void MotorRigTest::publish_actuator_test(int motor_index_1based, uint8_t action, float value)
{
	actuator_test_s test{};
	test.timestamp = hrt_absolute_time();
	test.function = actuator_test_s::FUNCTION_MOTOR1 + (motor_index_1based - 1);
	test.action = action;
	test.value = value;
	test.timeout_ms = (action == actuator_test_s::ACTION_DO_CONTROL) ? (uint32_t)_param_mrigt_timeout.get() : 0;
	_actuator_test_pub.publish(test);

	++_tests_published;
}

void MotorRigTest::set_logging_override(bool enable)
{
	char argv0[] = "logger";
	char argv1_on[] = "on";
	char argv1_off[] = "off";
	char *argv[] = {argv0, enable ? argv1_on : argv1_off, nullptr};
	logger_main(2, argv);
}

float MotorRigTest::motor_thrust_param(int motor_index_0based) const
{
	switch (motor_index_0based) {
	case 0: return _param_mrigt_m1_thr.get();
	case 1: return _param_mrigt_m2_thr.get();
	case 2: return _param_mrigt_m3_thr.get();
	default: return _param_mrigt_m4_thr.get();
	}
}

void MotorRigTest::Run()
{
	if (should_exit()) {
		// Release every motor still overridden so stopping the module
		// always leaves the bench in a safe (all motors off) state
		// immediately, rather than waiting out the last commanded timeout.
		for (int i = 0; i < NUM_MOTORS; ++i) {
			if (_active_mask & (1 << i)) {
				publish_actuator_test(i + 1, actuator_test_s::ACTION_RELEASE_CONTROL, NAN);
			}
		}

		_active_mask = 0;

		set_logging_override(false);

		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		parameters_updated();
	}

	// ------------------------------------------------------------
	// OFFBOARD safety gate
	//
	// This module is allowed to control the motor actuator-test
	// override only while the vehicle is in OFFBOARD mode.
	// If OFFBOARD is left, explicitly release every active motor
	// before stopping the module. This avoids waiting for the
	// actuator_test timeout to expire.
	// ------------------------------------------------------------
	vehicle_status_s status{};

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
	}

	const bool is_offboard =
		(_nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	if (!is_offboard) {
		PX4_INFO("OFFBOARD not active - stopping motor rig test");

		for (int i = 0; i < NUM_MOTORS; ++i) {
			if (_active_mask & (1 << i)) {
				publish_actuator_test(i + 1, actuator_test_s::ACTION_RELEASE_CONTROL, NAN);
			}
		}

		_active_mask = 0;

		set_logging_override(false);

		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	actuator_armed_s armed{};
	_actuator_armed_sub.copy(&armed);
	const bool is_armed = armed.armed || armed.kill;
	_was_armed = is_armed;

	if (is_armed) {
		// Defense in depth: MixingOutput already refuses to apply the
		// actuator_test override while armed, but this module must not even
		// ASK for one - so an operator who arms mid-test sees every motor
		// actually stop, not "still commanded but silently ignored".
		for (int i = 0; i < NUM_MOTORS; ++i) {
			if (_active_mask & (1 << i)) {
				publish_actuator_test(i + 1, actuator_test_s::ACTION_RELEASE_CONTROL, NAN);
			}
		}

		_active_mask = 0;
		++_early_return_armed;
		return;
	}

	const uint8_t requested_mask = _param_mrigt_motors.get() & ((1 << NUM_MOTORS) - 1);

	for (int i = 0; i < NUM_MOTORS; ++i) {
		const bool was_active = _active_mask & (1 << i);
		const bool now_active = requested_mask & (1 << i);

		if (was_active && !now_active) {
			// Bit cleared - release this motor immediately instead of
			// leaving it to coast until its own timeout.
			publish_actuator_test(i + 1, actuator_test_s::ACTION_RELEASE_CONTROL, NAN);

		} else if (now_active) {
			// Refreshed every cycle (whether newly set or continuing) so
			// the actuator_test timeout never lapses while this module is
			// healthy. Each motor uses its OWN independently-set thrust.
			const float value = math::constrain(motor_thrust_param(i), 0.f, 1.f);
			publish_actuator_test(i + 1, actuator_test_s::ACTION_DO_CONTROL, value);
		}
	}

	_active_mask = requested_mask;
}

int MotorRigTest::task_spawn(int argc, char *argv[])
{
	MotorRigTest *instance = new MotorRigTest();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			// This module deliberately never arms (it's a props-off bench
			// test), so PX4's default arm-gated logging would otherwise
			// never capture a run. Force a ulog open for the module's
			// lifetime instead; Run() releases the override on every exit
			// path.
			instance->set_logging_override(true);
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

int MotorRigTest::print_status()
{
	PX4_INFO("motors_mask=0x%x (bit0=M1..bit3=M4) timeout=%d ms armed=%d",
         static_cast<unsigned int>(_param_mrigt_motors.get()),
         static_cast<int>(_param_mrigt_timeout.get()),
         (int)_was_armed);

	PX4_INFO("thrust: M1=%.2f M2=%.2f M3=%.2f M4=%.2f",
		 (double)_param_mrigt_m1_thr.get(),
		 (double)_param_mrigt_m2_thr.get(),
		 (double)_param_mrigt_m3_thr.get(),
		 (double)_param_mrigt_m4_thr.get());

	PX4_INFO("active_mask=0x%x tests_published=%" PRIu64 " early_return_armed=%" PRIu64,
		 _active_mask, _tests_published, _early_return_armed);

	PX4_INFO("nav_state=%u offboard=%d", (unsigned)_nav_state,
		 (int)(_nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD));

	return 0;
}

int MotorRigTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MotorRigTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`motor_rig_test` is a standalone bench/ground-test tool that spins any
combination of this quad's 4 motors, each at its own independently-set
thrust value - it never computes a control setpoint and never touches the
position, velocity, attitude or rate controllers, or the control allocator.
It drives the motors through the same `actuator_test` uORB path as the
`actuator_test` command (see `ActuatorTest::overrideValues()` in
src/lib/mixer_module), which is only ever applied while the vehicle is
DISARMED, and only to the output channels put into test mode - every motor
whose bit is not set is left at its normal disarmed (off) value.

WARNING: remove all propellers (or use a fixture that physically restrains
the motors) before using this on real hardware. This module is a bench test
tool, not a flight controller - it refuses to command anything while the
vehicle is armed, releasing every active motor immediately if it detects an
arming edge.

Select which motor(s) spin with the `MRIGT_MOTORS` bitmask (bit 0 = motor 1
.. bit 3 = motor 4 - tick any combination in QGroundControl, or set the
decimal sum via `param set`), and each motor's own commanded thrust with
`MRIGT_M1_THR`..`MRIGT_M4_THR`. Clearing a bit while the module is running
immediately releases that motor. Start it directly with `motor_rig_test
start` from the shell.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_rig_test", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int motor_rig_test_main(int argc, char *argv[])
{
	return ModuleBase::main(MotorRigTest::desc, argc, argv);
}
