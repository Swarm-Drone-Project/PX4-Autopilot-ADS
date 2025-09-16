#pragma once

#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
// #include <uORB/SubscriptionCallbackWorkItem.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>

// Sensor Data, unfiltered and filtered
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/parameter_update.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;
using namespace matrix;

class TestFilter : public ModuleBase<TestFilter>, public ModuleParams, public px4::WorkItem
{
public:
	TestFilter();
	~TestFilter() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// /**
	//  * Check for parameter updates and update them if needed
	//  */
	// void updateParams();

	/**
	 * Complementary filter that fuses accelerometer and gyroscope data
	 * to estimate attitude (roll and pitch)
	 */
	void updateComplementaryFilter(const Vector3f &accel, const Vector3f &gyro, float dt);

	// Subscriptions
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _sensor_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// Publications
	// uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};

	// Filter state
	float _roll{0.0f};    // Roll angle estimate (rad)
	float _pitch{0.0f};   // Pitch angle estimate (rad)
	float _yaw{0.0f};     // Yaw angle estimate (rad)

	// Timing
	hrt_abstime _last_update{0};
	
	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _loop_interval_perf;

	// Filter parameter
	const float _alpha{0.98f}; // Complementary filter coefficient (tune as needed)
	// Parameters
	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::TF_ALPHA>) _param_alpha,
	// 	(ParamInt<px4::params::TF_UPDATE_RATE>) _param_update_rate,
	// 	(ParamInt<px4::params::TF_DEBUG>) _param_debug
	// )
};
