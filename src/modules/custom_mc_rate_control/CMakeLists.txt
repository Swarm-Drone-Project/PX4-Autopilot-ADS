px4_add_module(
    MODULE modules__custom_mc_rate_control
    MAIN custom_mc_rate_control
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}
    SRCS
        custom_mc_rate_control.cpp
        custom_mc_rate_control.hpp
    DEPENDS
        mathlib
        px4_work_queue
)
