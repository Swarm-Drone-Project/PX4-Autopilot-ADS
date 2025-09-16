px4_add_module(
	MODULE modules__test_filter
	MAIN test_filter
	COMPILE_FLAGS
	SRCS
		test_filter_main.cpp
		test_filter.hpp
	DEPENDS
		mathlib
		px4_work_queue
	)
