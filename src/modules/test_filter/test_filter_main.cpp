#include "test_filter.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

TestFilter::TestFilter() :
	ModuleParams(nullptr),
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_loop_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

TestFilter::~TestFilter()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool TestFilter::init()
{
	// ScheduleOnInterval(25_ms); // 1000 Hz
	// Register callback instead of polling
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void TestFilter::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Get current time
	const hrt_abstime now = hrt_absolute_time();
	const float dt = (_last_update > 0) ? (now - _last_update) / 1e6f : 0.02f; // Default to 50Hz if first run
	PX4_INFO("dt: %.6f", (double)dt);
	_last_update = now;

	// Read sensor data
	sensor_accel_s accel_data;
	sensor_gyro_s gyro_data;

	bool accel_updated = _sensor_accel_sub.update(&accel_data);
	bool gyro_updated = _sensor_gyro_sub.update(&gyro_data);

	if (accel_updated && gyro_updated) {
		// Convert to Vector3f for easier math operations
		Vector3f accel(accel_data.x, accel_data.y, accel_data.z);
		Vector3f gyro(gyro_data.x, gyro_data.y, gyro_data.z);

		// Update complementary filter
		updateComplementaryFilter(accel, gyro, dt);
		PX4_INFO("Roll: %.3f, Pitch: %.3f, Yaw: %.3f", (double)_roll, (double)_pitch, (double)_yaw);
		// Publish attitude estimate
		// vehicle_attitude_s attitude{};
		// attitude.timestamp = now;
		
		// Convert Euler angles to quaternion
		// Using aerospace sequence: roll, pitch, yaw (Z-Y-X)
		// Eulerf euler(_roll, _pitch, _yaw);
		// Quatf q(euler);
		
		// attitude.q[0] = q(0);
		// attitude.q[1] = q(1);
		// attitude.q[2] = q(2);
		// attitude.q[3] = q(3);

		// // Set timestamp sample to match the sensor data
		// attitude.timestamp_sample = accel_data.timestamp_sample;

		// _attitude_pub.publish(attitude);
	}
	// px4_usleep(100); // Sleep for 100 microseconds to yield CPU
	perf_end(_loop_perf);
	perf_print_counter(_loop_perf);
}

void TestFilter::updateComplementaryFilter(const Vector3f &accel, const Vector3f &gyro, float dt)
{
	// Complementary Filter Implementation
	// High-pass filter on gyroscope data, low-pass filter on accelerometer data
	
	// Step 1: Integrate gyroscope data (high-frequency, short-term accurate)
	float roll_gyro = _roll + gyro(0) * dt;   // p (roll rate)
	float pitch_gyro = _pitch + gyro(1) * dt; // q (pitch rate)
	
	// Step 2: Calculate roll and pitch from accelerometer (low-frequency, long-term accurate)
	// Assuming no significant acceleration other than gravity
	float roll_accel = atan2f(accel(1), accel(2));  // atan2(ay, az)
	float pitch_accel = atan2f(-accel(0), sqrtf(accel(1) * accel(1) + accel(2) * accel(2))); // atan2(-ax, sqrt(ay^2 + az^2))
	
	// Step 3: Complementary filter fusion
	// _alpha = 0.98 means 98% gyro (high frequency) + 2% accel (low frequency)
	_roll = _alpha * roll_gyro + (1.0f - _alpha) * roll_accel;
	_pitch = _alpha * pitch_gyro + (1.0f - _alpha) * pitch_accel;
	
	// Note: Yaw cannot be estimated from accelerometer alone (no magnetometer in this simple filter)
	// For now, just integrate gyro yaw rate
	_yaw += gyro(2) * dt; // r (yaw rate)
	
	// Keep yaw in [-pi, pi] range
	if (_yaw > M_PI_F) {
		_yaw -= 2.0f * M_PI_F;
	} else if (_yaw < -M_PI_F) {
		_yaw += 2.0f * M_PI_F;
	}
}

int TestFilter::task_spawn(int argc, char *argv[])
{
	TestFilter *instance = new TestFilter();

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

int TestFilter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simple complementary filter that fuses accelerometer and gyroscope data to estimate attitude.

The filter uses a weighted average of:
- Gyroscope integration (high-frequency, short-term accurate)
- Accelerometer-derived angles (low-frequency, long-term accurate)

Filter coefficient alpha = 0.98 (98% gyro, 2% accel)

### Implementation
Runs at 100Hz using the work queue system.
Publishes vehicle_attitude topic with estimated roll, pitch, and yaw.

Note: Yaw estimation is limited without magnetometer data.

### Examples
Start the complementary filter:
$ test_filter start

Stop the filter:
$ test_filter stop

Check status:
$ test_filter status

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("test_filter", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TestFilter::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

extern "C" __EXPORT int test_filter_main(int argc, char *argv[])
{
	return TestFilter::main(argc, argv);
}
