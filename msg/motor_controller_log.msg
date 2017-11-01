# Definition of the motor_controller_log topic

float32 throttle	# Value going to the actuator
float32 rpm
bool started
bool kill_switch
int16 switch_state
float32 filtered_rpm_command
float32 throttle_offset
float32 dt