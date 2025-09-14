px4_add_module(
    MODULE modules__testing_control
    MAIN testing_control
    SRCS
        TestingControl.cpp
    DEPENDS
        px4_work_queue
)
