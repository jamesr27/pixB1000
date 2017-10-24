
include_directories(${PX4_SOURCE_DIR}/mavlink/include/mavlink)

px4_add_module(
	MODULE modules__rotor_rpm_reader
	MAIN rotor_rpm_reader
	PRIORITY "SCHED_PRIORITY_MAX-40"
	STACK_MAIN 1500
	STACK_MAX 2000
	COMPILE_FLAGS -Os
	SRCS
		rotor_rpm_reader.cpp
		my_serial2.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
