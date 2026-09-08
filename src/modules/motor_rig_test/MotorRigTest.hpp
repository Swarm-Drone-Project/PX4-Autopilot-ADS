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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

// Standalone bench-test module for spinning any combination of this quad's 4
// motors, each at its own independently-set thrust value, selected by a
// bitmask parameter (MRIGT_MOTORS) instead of a CLI invocation. It never
// computes a control setpoint of its own and never touches the
// position/velocity/attitude/rate controllers or the control allocator - it
// drives the motor(s) through the same actuator_test uORB path as the
// `actuator_test` command (see src/systemcmds/actuator_test and
// ActuatorTest::update()/overrideValues() in src/lib/mixer_module). That
// path overrides one output channel's value downstream of the mixer, ONLY
// while disarmed (MixingOutput gates overrideValues() on actuator_armed's
// !armed && !kill), and any channel this module does NOT put in test mode
// is fed NaN by ActuatorTest, which output_limit_calc_single() maps to that
// channel's disarmed (off) value - so exactly the motors whose bit is set
// spin, and every other motor stays off. This is a bench/ground test tool:
// it refuses to command anything while the vehicle is armed, props must be
// removed before use on real hardware, and it is not meant to fly. Start it
// directly with `motor_rig_test start` and select motors with MRIGT_MOTORS
// (bit 0 = motor 1 .. bit 3 = motor 4), each spinning at its own
// MRIGT_M<n>_THR value.
class MotorRigTest : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	MotorRigTest();
	~MotorRigTest() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();
	void publish_actuator_test(int motor_index_1based, uint8_t action, float value);
	float motor_thrust_param(int motor_index_0based) const;

	// Forces the logger to open (enable=true) or release (enable=false) a
	// ulog file regardless of arm state, via the same manual override the
	// `logger on` / `logger off` shell commands use. Needed because this
	// module intentionally never arms (it refuses to command anything while
	// armed) - without this, PX4's default arm-gated logging would never
	// capture a run of this test.
	void set_logging_override(bool enable);

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};

	static constexpr int NUM_MOTORS = 4;

	// Bit i (0-based) = motor (i+1) currently overridden by this module.
	// Tracked so a bit clearing (or a rising armed edge) can explicitly
	// release just that motor instead of relying solely on the
	// actuator_test timeout to stop it.
	uint8_t _active_mask{0};
	bool _was_armed{false};

	// Motor rig test is allowed to operate only while the vehicle is in
	// OFFBOARD mode. Any other navigation state causes an immediate
	// release of all active actuator-test overrides and module shutdown.
	uint8_t _nav_state{vehicle_status_s::NAVIGATION_STATE_MANUAL};

	uint64_t _tests_published{0};
	uint64_t _early_return_armed{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 20};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MRIGT_MOTORS>) _param_mrigt_motors,
		(ParamFloat<px4::params::MRIGT_M1_THR>) _param_mrigt_m1_thr,
		(ParamFloat<px4::params::MRIGT_M2_THR>) _param_mrigt_m2_thr,
		(ParamFloat<px4::params::MRIGT_M3_THR>) _param_mrigt_m3_thr,
		(ParamFloat<px4::params::MRIGT_M4_THR>) _param_mrigt_m4_thr,
		(ParamInt<px4::params::MRIGT_TIMEOUT>) _param_mrigt_timeout,
		(ParamInt<px4::params::MRIGT_PUB_HZ>) _param_mrigt_pub_hz
	)
};
