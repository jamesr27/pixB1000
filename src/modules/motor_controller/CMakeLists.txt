
px4_add_module(
	MODULE modules__motor_controller
	MAIN motor_controller
	PRIORITY "SCHED_PRIORITY_MAX-40"
	STACK_MAIN 2500
	STACK_MAX 3500
	COMPILE_FLAGS -Os
	SRCS
		motor_controller.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
