#include "custom_mc_rate_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

using namespace matrix;

CustomMCRateControl::CustomMCRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_loop_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME": interval");
}

CustomMCRateControl::~CustomMCRateControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CustomMCRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void CustomMCRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = angular_velocity.timestamp_sample;
		const float dt = math::constrain((now - _last_run) * 1e-6f, 0.0002f, 0.02f);
		_last_run = now;

		// Update control mode
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		// Get rate setpoint
		vehicle_rates_setpoint_s rates_sp;
		if (_vehicle_rates_setpoint_sub.update(&rates_sp)) {
			_rates_setpoint = Vector3f(rates_sp.roll, rates_sp.pitch, rates_sp.yaw);
			_thrust_setpoint = Vector3f(rates_sp.thrust_body);
		}

		// Reset integrator if rates control disabled
		if (!_vehicle_control_mode.flag_control_rates_enabled) {
			_rate_int.zero();
			perf_end(_loop_perf);
			return;
		}

		// Current rates and angular acceleration
		const Vector3f rates(angular_velocity.xyz);
		const Vector3f angular_accel(angular_velocity.xyz_derivative);

		// Rate error
		const Vector3f rate_error = _rates_setpoint - rates;

		// PID control (inline for minimal overhead)
		Vector3f torque;
		torque(0) = ROLL_P  * rate_error(0) + _rate_int(0) - ROLL_D  * angular_accel(0);
		torque(1) = PITCH_P * rate_error(1) + _rate_int(1) - PITCH_D * angular_accel(1);
		torque(2) = YAW_P   * rate_error(2) + _rate_int(2) - YAW_D   * angular_accel(2);

		// Non-linear I factor: reduces integral gain at high rate errors
		// Prevents bounce-back after aggressive maneuvers (flips, rolls)
		// At 400 deg/s error -> i_factor ≈ 0, at 100 deg/s -> i_factor ≈ 0.94
		auto i_factor = [](float err) {
			constexpr float scale = 1.f / 6.98f;  // 1 / radians(400)
			float f = err * scale;
			return math::max(0.0f, 1.f - f * f);
		};

		// Update integral with non-linear reduction and anti-windup clamp
		_rate_int(0) = math::constrain(_rate_int(0) + i_factor(fabsf(rate_error(0))) * ROLL_I  * rate_error(0) * dt, -INT_LIM, INT_LIM);
		_rate_int(1) = math::constrain(_rate_int(1) + i_factor(fabsf(rate_error(1))) * PITCH_I * rate_error(1) * dt, -INT_LIM, INT_LIM);
		_rate_int(2) = math::constrain(_rate_int(2) + i_factor(fabsf(rate_error(2))) * YAW_I   * rate_error(2) * dt, -INT_LIM, INT_LIM);

		// Publish torque
		vehicle_torque_setpoint_s torque_msg{};
		torque_msg.timestamp_sample = angular_velocity.timestamp_sample;
		torque_msg.timestamp = hrt_absolute_time();
		torque.copyTo(torque_msg.xyz);
		_vehicle_torque_setpoint_pub.publish(torque_msg);

		// Publish thrust (passthrough)
		vehicle_thrust_setpoint_s thrust_msg{};
		thrust_msg.timestamp_sample = angular_velocity.timestamp_sample;
		thrust_msg.timestamp = hrt_absolute_time();
		_thrust_setpoint.copyTo(thrust_msg.xyz);
		_vehicle_thrust_setpoint_pub.publish(thrust_msg);
	}

	perf_end(_loop_perf);
}

int CustomMCRateControl::task_spawn(int argc, char *argv[])
{
	CustomMCRateControl *instance = new CustomMCRateControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int CustomMCRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomMCRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal rate controller for X500. Hardcoded PID gains, no parameter overhead.
Publishes vehicle_torque_setpoint and vehicle_thrust_setpoint.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_mc_rate_control_main(int argc, char *argv[])
{
	return CustomMCRateControl::main(argc, argv);
}
