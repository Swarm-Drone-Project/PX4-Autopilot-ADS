#include "custom_mc_att_control.hpp"

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace matrix;

CustomAttitudeControl::CustomAttitudeControl() :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _loop_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
    // Initialize for X500 quadcopter - optimized for Gazebo simulation
    PX4_INFO("Custom Attitude Controller initialized for Gazebo X500");
}

CustomAttitudeControl::~CustomAttitudeControl()
{
    // Clean up performance counters
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool CustomAttitudeControl::init()
{
    // Register callback for attitude updates
    if (!_vehicle_attitude_sub.registerCallback()) {
        PX4_ERR("vehicle_attitude callback registration failed!");
        return false;
    }

    return true;
}

void CustomAttitudeControl::Run()
{
    if (should_exit()) {
	_vehicle_attitude_sub.unregisterCallback();
	exit_and_cleanup();
	return;
    }
//     PX4_INFO("CustomAttitudeControl::Run() called");
    // CRITICAL: Start performance measurement
    perf_begin(_loop_perf);

    // Get current attitude
    vehicle_attitude_s vehicle_attitude;
    if (_vehicle_attitude_sub.update(&vehicle_attitude)) {

	const hrt_abstime now = vehicle_attitude.timestamp_sample;
	const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);;
	_last_run = now;

	// Current attitude quaternion
	const Quatf q_current(vehicle_attitude.q);

	// Get attitude setpoint (default to level if no setpoint)
	vehicle_attitude_setpoint_s attitude_setpoint{};
	Quatf q_setpoint(1.0f, 0.0f, 0.0f, 0.0f); // Default: level attitude

	// Initialize default thrust (X500 hover thrust ~ -0.5 in NED frame)
	attitude_setpoint.thrust_body[0] = 0.0f;   // No forward thrust
	attitude_setpoint.thrust_body[1] = 0.0f;   // No sideways thrust
	attitude_setpoint.thrust_body[2] = -0.8f;  // Default hover thrust (negative = upward in NED)
	if (_vehicle_attitude_setpoint_sub.updated()) {
		if (_vehicle_attitude_setpoint_sub.copy(&attitude_setpoint)) {
			q_setpoint = Quatf(attitude_setpoint.q_d);
			// Thrust values will be from the setpoint
		}
	}

	// Calculate attitude error
	const Quatf q_error = q_setpoint * q_current.inversed();

	// Convert quaternion error to axis-angle (small angle approximation)
	Vector3f attitude_error;
	if (q_error(0) >= 0.0f) {
		attitude_error = 2.0f * Vector3f(q_error(1), q_error(2), q_error(3));
	} else {
		attitude_error = -2.0f * Vector3f(q_error(1), q_error(2), q_error(3));
	}

	// Simple PID control
	Vector3f rates_setpoint = pid_controller(attitude_error, dt);

	// Publish rate setpoint
	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp = now;
	rates_sp.roll = rates_setpoint(0);
	rates_sp.pitch = rates_setpoint(1);
	rates_sp.yaw = rates_setpoint(2);

	// Use thrust from attitude setpoint (essential for flight!)
	rates_sp.thrust_body[0] = attitude_setpoint.thrust_body[0];  // Forward thrust
	rates_sp.thrust_body[1] = attitude_setpoint.thrust_body[1];  // Right thrust
	rates_sp.thrust_body[2] = attitude_setpoint.thrust_body[2];  // Down thrust (negative = up)

	_vehicle_rates_setpoint_pub.publish(rates_sp);

    }

    // CRITICAL: Calculate time delta with high precision
    const hrt_abstime now = hrt_absolute_time();
    const float dt = (_last_run != 0) ? (now - _last_run) * 1e-6f : ATTITUDE_CTRL_EXPECTED_DT;
    _last_run = now;

    // CRITICAL: Measure interval between executions
    perf_count(_loop_interval_perf);

    // CRITICAL: Guard against bad dt - this could cause instability or oscillations
    if (dt < ATTITUDE_CTRL_MIN_DT || dt > ATTITUDE_CTRL_MAX_DT) {
        PX4_WARN("CRITICAL: Bad dt: %.6f ms (expected: %.1f ms) - SKIPPING CONTROL CYCLE",
                 (double)(dt * 1000.0f), (double)(ATTITUDE_CTRL_EXPECTED_DT * 1000.0f));
        perf_end(_loop_perf);
        return;
    }

    // CRITICAL: Warn if timing is degraded but still acceptable
    if (dt > ATTITUDE_CTRL_EXPECTED_DT * 1.5f) {
        static uint64_t last_warn = 0;
        if (now - last_warn > 1000000) { // Warn max once per second
            PX4_WARN("Degraded timing: %.2f ms (expected: %.1f ms)",
                     (double)(dt * 1000.0f), (double)(ATTITUDE_CTRL_EXPECTED_DT * 1000.0f));
            last_warn = now;
        }
    }

    // Check if attitude control is enabled (disabled during offboard rate control)
    _vehicle_control_mode_sub.update(&_vehicle_control_mode);

    if (!_vehicle_control_mode.flag_control_attitude_enabled) {
        // Attitude control disabled - external source providing rate setpoints
        perf_end(_loop_perf);
        return;
    }
    // Current attitude quaternion
    const Quatf q_current(vehicle_attitude.q);

    // Get attitude setpoint (default to level if no setpoint)
    vehicle_attitude_setpoint_s attitude_setpoint{};
    Quatf q_setpoint(1.0f, 0.0f, 0.0f, 0.0f); // Default: level attitude

    // Initialize default thrust (X500 hover thrust ~ -0.5 in NED frame)
    attitude_setpoint.thrust_body[0] = 0.0f;   // No forward thrust
    attitude_setpoint.thrust_body[1] = 0.0f;   // No sideways thrust
    attitude_setpoint.thrust_body[2] = -0.5f;  // Default hover thrust (negative = upward in NED)

    if (_vehicle_attitude_setpoint_sub.copy(&attitude_setpoint)) {
        q_setpoint = Quatf(attitude_setpoint.q_d);
        // Thrust values will be from the setpoint
    }

    // Calculate attitude error
    const Quatf q_error = q_setpoint * q_current.inversed();

    // Convert quaternion error to axis-angle (small angle approximation)
    Vector3f attitude_error;
    if (q_error(0) >= 0.0f) {
        attitude_error = 2.0f * Vector3f(q_error(1), q_error(2), q_error(3));
    } else {
        attitude_error = -2.0f * Vector3f(q_error(1), q_error(2), q_error(3));
    }

    // Simple PID control
    Vector3f rates_setpoint = pid_controller(attitude_error, dt);

    // Publish rate setpoint
    vehicle_rates_setpoint_s rates_sp{};
    rates_sp.timestamp = now;
    rates_sp.roll = rates_setpoint(0);
    rates_sp.pitch = rates_setpoint(1);
    rates_sp.yaw = rates_setpoint(2);

    // Use thrust from attitude setpoint (essential for flight!)
    rates_sp.thrust_body[0] = attitude_setpoint.thrust_body[0];  // Forward thrust
    rates_sp.thrust_body[1] = attitude_setpoint.thrust_body[1];  // Right thrust
    rates_sp.thrust_body[2] = attitude_setpoint.thrust_body[2];  // Down thrust (negative = up)

    _vehicle_rates_setpoint_pub.publish(rates_sp);

    // CRITICAL: End performance measurement
    perf_end(_loop_perf);
}

Vector3f CustomAttitudeControl::pid_controller(const Vector3f &error, float dt)
{
    // Proportional term (using PX4 parameters optimized for X500)
    Vector3f proportional;
    proportional(0) = _param_roll_p.get() * error(0);   // Roll  - typically ~6.0 for X500
    proportional(1) = _param_pitch_p.get() * error(1);  // Pitch - typically ~6.0 for X500
    proportional(2) = _param_yaw_p.get() * error(2);    // Yaw   - typically ~2.8 for X500

    // Integral term (simple integration with X500-tuned gain)
    _attitude_error_integral += error * dt;

    // Anti-windup: limit integral (X500-specific limits)
    _attitude_error_integral(0) = math::constrain(_attitude_error_integral(0), -0.5f, 0.5f);
    _attitude_error_integral(1) = math::constrain(_attitude_error_integral(1), -0.5f, 0.5f);
    _attitude_error_integral(2) = math::constrain(_attitude_error_integral(2), -0.3f, 0.3f);

    Vector3f integral = _attitude_error_integral * 0.15f; // X500-tuned I gain

    // Derivative term (X500-tuned D gain)
    Vector3f derivative = (error - _attitude_error_previous) / dt * 0.08f; // Higher D for X500 stability
    _attitude_error_previous = error;

    // PID output (rate setpoint in rad/s)
    Vector3f output = proportional + integral + derivative;

    // Limit output rates (X500-specific limits)
    Vector3f constrained_output;
    constrained_output(0) = math::constrain(output(0), -X500_MAX_ROLL_RATE, X500_MAX_ROLL_RATE);   // Roll: ±172°/s
    constrained_output(1) = math::constrain(output(1), -X500_MAX_PITCH_RATE, X500_MAX_PITCH_RATE); // Pitch: ±172°/s
    constrained_output(2) = math::constrain(output(2), -X500_MAX_YAW_RATE, X500_MAX_YAW_RATE);     // Yaw: ±86°/s

    return constrained_output;
}

// Module spawn function
int CustomAttitudeControl::task_spawn(int argc, char *argv[])
{
    // X500 is a pure quadcopter - no VTOL support needed
    CustomAttitudeControl *instance = new CustomAttitudeControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            PX4_INFO("Custom Attitude Control started for X500");
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

void CustomAttitudeControl::print_perf()
{
    PX4_INFO("Performance:");
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
}

int CustomAttitudeControl::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_WARN("not running");
        return 1;
    }

    if (!strcmp(argv[0], "perf")) {
        get_instance()->print_perf();
        return 0;
    }

    return print_usage("unknown command");
}

int CustomAttitudeControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Minimal attitude controller optimized for Gazebo X500 quadcopter research.
Takes attitude setpoints and outputs rate setpoints using simple PID control.

This is a simplified version of mc_att_control designed specifically for the X500 model
in Gazebo simulation. It provides basic attitude stabilization with X500-specific limits.

### X500 Optimization
- Roll/Pitch rate limit: ±3.0 rad/s (±172°/s)
- Yaw rate limit: ±1.5 rad/s (±86°/s)
- Optimized for X500 mass and inertia characteristics
- No VTOL complexity - pure quadcopter operation

CRITICAL TIMING:
- Expected rate: 250Hz (4ms intervals)
- Acceptable range: 50Hz to 1000Hz (1ms to 20ms)
- Performance monitoring via perf counters

### Performance Monitoring
Use 'custom_mc_att_control perf' to check timing performance.
Critical for flight control - bad timing can cause instability!
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("custom_mc_att_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("perf", "Print performance counters");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// Main entry point
extern "C" __EXPORT int custom_mc_att_control_main(int argc, char *argv[])
{
    return CustomAttitudeControl::main(argc, argv);
}
