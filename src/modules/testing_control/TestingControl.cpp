#include "TestingControl.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/events.h>

#include <cstring>

using namespace time_literals;

TestingControl::TestingControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem("testing_control", px4::wq_configurations::rate_ctrl)
{
    // Initialize all motors to zero
    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        _motor_speeds[i] = 0.0f;
    }
}

TestingControl::~TestingControl()
{
    stop_all_motors();
}

void TestingControl::stop_all_motors()
{
    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        set_motor_speed(i, 0.0f);
    }
    _armed = false;
}

bool TestingControl::parse_command(int argc, char *argv[])
{
    // This function is deprecated - all command handling is now in custom_command
    return false;
}

void TestingControl::set_motor_speed(uint8_t motor_idx, float value)
{
    if (!_armed && value > 0.0f) {
        PX4_WARN("Motors not armed!");
        return;
    }

    // Create and fill actuator outputs message
    actuator_outputs_s actuator_outputs{};
    actuator_outputs.timestamp = hrt_absolute_time();
    actuator_outputs.noutputs = MAX_MOTORS;

    // Copy current motor speeds and scale them for simulation (0-1 to 0-1)
    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        actuator_outputs.output[i] = _motor_speeds[i];
    }

    // Update specific motor
    actuator_outputs.output[motor_idx] = value;

    // Publish to both regular and simulation topics
    _actuator_outputs_pub.publish(actuator_outputs);
    _actuator_outputs_sim_pub.publish(actuator_outputs);

//     PX4_INFO("Published actuator output: motor %d = %.2f", motor_idx, (double)value);
}

void TestingControl::Run()
{
    if (should_exit()) {
        stop_all_motors();
        exit_and_cleanup();
        return;
    }

    // Only update motors if armed and at least one motor has non-zero speed
    bool should_update = false;
    if (_armed) {
        for (uint8_t i = 0; i < MAX_MOTORS; i++) {
            if (_motor_speeds[i] > 0.0f) {
                should_update = true;
                break;
            }
        }
    }

    if (should_update) {
        // Update all motor outputs
        for (uint8_t i = 0; i < MAX_MOTORS; i++) {
            set_motor_speed(i, _motor_speeds[i]);
        }
    }

    // Schedule next run
    ScheduleDelayed(200_ms);
}

int TestingControl::task_spawn(int argc, char *argv[])
{
    TestingControl *instance = new TestingControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        instance->ScheduleNow();
        return PX4_OK;
    }

    delete instance;
    return PX4_ERROR;
}

int TestingControl::custom_command(int argc, char *argv[])
{
    TestingControl *instance = _object.load();
    PX4_INFO("Custom command received argc=%d argv[0]=%s", argc, argv[0]);

    if (!instance) {
        PX4_ERR("Module not running. Please start with 'testing_control start' first");
        return -1;
    }

    for (int i = 0; i < argc; i++) {
        PX4_DEBUG("argv[%d] = %s", i, argv[i]);
    }

    const char *command = (argc > 0) ? argv[0] : nullptr;
    if (!command) {
        return print_usage("missing command");
    }

    if (!strcmp(command, "arm")) {
        instance->_armed = true;
        PX4_INFO("Motors armed");
        return 0;
    }

    if (!strcmp(command, "disarm")) {
        instance->stop_all_motors();
        PX4_INFO("Motors disarmed");
        return 0;
    }

    if (!strcmp(command, "motor")) {
        if (argc != 3) {
            PX4_ERR("Usage: motor <index> <value>");
            return -1;
        }

        uint8_t motor = atoi(argv[1]);
        float value = atof(argv[2]);

        if (motor >= TestingControl::MAX_MOTORS) {
            PX4_ERR("Motor index must be between 0 and %d", TestingControl::MAX_MOTORS - 1);
            return -1;
        }

        if (value < 0.0f || value > 1.0f) {
            PX4_ERR("Motor value must be between 0.0 and 1.0");
            return -1;
        }

        instance->_motor_speeds[motor] = value;
        // The Run() function will now handle the publishing
        PX4_INFO("Motor %d speed set to %.2f, will be published in next cycle.", motor, (double)value);
        return 0;
    }

    return print_usage("unknown command");
}
int TestingControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Testing control module for direct motor control

### Examples
$ testing_control arm           # Arm motors
$ testing_control disarm        # Disarm motors
$ testing_control motor 0 0.5   # Set motor 0 to 50% throttle
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("testing_control", "command");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the module");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print module status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Arm motors");
    PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Disarm motors");
    PRINT_MODULE_USAGE_COMMAND_DESCR("motor <index> <value>", "Control specific motor (index: 0-7, value: 0.0-1.0)");

    return 0;
}

extern "C" __EXPORT int testing_control_main(int argc, char *argv[])
{
    PX4_INFO("Main function called with command: %s", (argc > 1) ? argv[1] : "none");
    return TestingControl::main(argc, argv);
}
