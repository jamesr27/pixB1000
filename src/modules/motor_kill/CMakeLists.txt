
px4_add_module(
	MODULE modules__motor_kill
	MAIN motor_kill
	PRIORITY "SCHED_PRIORITY_MAX-40"
	STACK_MAIN 1000
	STACK_MAX 2000
	COMPILE_FLAGS -Os
	SRCS
		motor_kill.cpp
		my_serial.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
