#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class CustomMCRateControl : public ModuleBase<CustomMCRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	CustomMCRateControl();
	~CustomMCRateControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Subscriptions
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Publications
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// State
	matrix::Vector3f _rates_setpoint{};
	matrix::Vector3f _thrust_setpoint{};
	matrix::Vector3f _rate_int{};  // PID integral term
	vehicle_control_mode_s _vehicle_control_mode{};

	hrt_abstime _last_run{0};
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _loop_interval_perf{nullptr};

	// X500 tuned PID gains (hardcoded for minimal overhead)
	static constexpr float ROLL_P  = 0.15f;
	static constexpr float ROLL_I  = 0.2f;
	static constexpr float ROLL_D  = 0.003f;
	static constexpr float PITCH_P = 0.22f;   // Increased for faster flip tracking. Earlier 0.15
	static constexpr float PITCH_I = 0.2f;
	static constexpr float PITCH_D = 0.003f;
	static constexpr float YAW_P   = 0.2f;
	static constexpr float YAW_I   = 0.1f;
	static constexpr float YAW_D   = 0.0f;
	static constexpr float INT_LIM = 0.4f;    // Increased for aggressive maneuvers. Earlier 0.3f
};
