#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>

class TestingControl : public ModuleBase<TestingControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    TestingControl();
    ~TestingControl() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void Run() override;

private:
    bool parse_command(int argc, char *argv[]);
    void set_motor_speed(uint8_t motor_idx, float value);
    void stop_all_motors();

    // Publications
    uORB::Publication<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};
    uORB::Publication<actuator_outputs_s> _actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};

    // Motor states
    static constexpr uint8_t MAX_MOTORS = 8;
    float _motor_speeds[MAX_MOTORS] = {};
    bool _armed{false};
};
