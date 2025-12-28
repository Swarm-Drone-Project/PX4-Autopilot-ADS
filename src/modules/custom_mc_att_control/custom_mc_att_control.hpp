#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

// CRITICAL TIMING CONSTANTS - Optimized for Gazebo X500
static constexpr float ATTITUDE_CTRL_EXPECTED_DT = 0.004f; // 250Hz expected rate
static constexpr float ATTITUDE_CTRL_MIN_DT = 0.001f;      // 1000Hz maximum rate
static constexpr float ATTITUDE_CTRL_MAX_DT = 0.02f;       // 50Hz minimum rate

// X500 SPECIFIC CONFIGURATION
static constexpr float X500_MAX_ROLL_RATE = 3.0f;   // X500 max roll rate (rad/s) - ~172°/s
static constexpr float X500_MAX_PITCH_RATE = 3.0f;  // X500 max pitch rate (rad/s) - ~172°/s
static constexpr float X500_MAX_YAW_RATE = 1.5f;    // X500 max yaw rate (rad/s) - ~86°/s

class CustomAttitudeControl : public ModuleBase<CustomAttitudeControl>, public ModuleParams, public px4::WorkItem
{
public:
    CustomAttitudeControl();
    ~CustomAttitudeControl();

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    // Simple PID controller for attitude
    matrix::Vector3f pid_controller(const matrix::Vector3f &error, float dt);

    // Performance reporting
    void print_perf();

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

    // Publications
    uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

    // Controller state
    matrix::Vector3f _attitude_error_integral{0.f, 0.f, 0.f};
    matrix::Vector3f _attitude_error_previous{0.f, 0.f, 0.f};
    hrt_abstime _last_run{0};

    // Cached data from subscriptions
    vehicle_control_mode_s _vehicle_control_mode{};

    // Performance monitoring (critical for flight control)
    perf_counter_t _loop_perf;          ///< loop performance counter
    perf_counter_t _loop_interval_perf; ///< interval performance counter

    // Simple parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::MC_ROLL_P>)  _param_roll_p,
        (ParamFloat<px4::params::MC_PITCH_P>) _param_pitch_p,
        (ParamFloat<px4::params::MC_YAW_P>)   _param_yaw_p
    )
};
