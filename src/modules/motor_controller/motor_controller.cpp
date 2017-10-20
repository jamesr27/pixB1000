/* James Reeves. Motor (engine) controller for the B1000 helicopter.
This does a few things:
	1. Receives motor armed commands. From the standard uORB.
	2. Receives the throttle command. This comes from the rc at present (channel 9, [8] in zero counting).
	3. Receives the rpm from the rpm sensor (through uORB). (Must add the filing of the rotor rpm mavlink message to mavlink still).
	4. Calculates the throttle output command (PID controller on throttle, or PD, or whatever works best). Published to uORB. We will assign the actuator value in the attitude controller or wherever this is done these days.)
*/

#include "motor_controller.h"


namespace motor_controller {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	MotorController *g_control;

 }

//Constructor
 MotorController::MotorController() :
 _task_should_exit(false),
 _control_task(-1),

 	 /* subscriptions */
 _rotor_rpm_sub(-1),
 _actuator_armed_sub(-1),
 _rc_channels_sub(-1),
 _vehicle_attitude_sub(-1),

	/* publications */
 _motor_throttle_pub(nullptr),
 _motor_kill_pub(nullptr),

 _param_counter(0),
 _integral_sum(0),
 _previous_error(0),
 _switch_state(0),
 _filtered_rpm_command(0.0f)


 {
 	memset(&_rotor_rpm, 0, sizeof(_rotor_rpm));
 	memset(&_motor_throttle, 0, sizeof(_motor_throttle));
 	memset(&_actuator_armed, 0, sizeof(_actuator_armed));
	memset(&_rc_channels, 0, sizeof(_rc_channels));
	memset(&_motor_kill, 0, sizeof(_motor_kill));

 	_params_handles.motor_control_p			= 	param_find("MOTOR_C_P");
 	_params_handles.motor_control_i			= 	param_find("MOTOR_C_I");
 	_params_handles.motor_control_d			= 	param_find("MOTOR_C_D");
 	_params_handles.motor_control_ilim		= 	param_find("MOTOR_C_ILIM");
 	_params_handles.motor_control_idleRpm	= 	param_find("MOTOR_C_IDLERPM");
 	_params_handles.motor_control_nomRpm	= 	param_find("MOTOR_C_NOMRPM");
 	_params_handles.motor_control_upSat		= 	param_find("MOTOR_C_UPSAT");
 	_params_handles.motor_control_lowSat	= 	param_find("MOTOR_C_LOWSAT");
 	_params_handles.motor_control_startRate	= 	param_find("MOTOR_C_SRATE");
 	_params_handles.motor_control_startRpm	= 	param_find("MOTOR_C_SRPM");
 	_params_handles.motor_control_rate		= 	param_find("MOTOR_C_RATE");

 	parameters_update();
 }


//Destructor
MotorController::~MotorController()
 {
 	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
 		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
 		unsigned i = 0;

 		do {
			/* wait 20ms */
 			usleep(20000);

			/* if we have given up, kill it */
 			if (++i > 50) {
 				px4_task_delete(_control_task);
 				break;
 			}
 		} while (_control_task != -1);
 	}

 	motor_controller::g_control = nullptr;

 }


//Parameter forced updates if system isn't listening
 int
 MotorController::parameters_update()
 {
	 if (_param_counter%500 == 0)
	 {
		param_get(_params_handles.motor_control_p, &_params.motor_control_p);
		param_get(_params_handles.motor_control_i, &_params.motor_control_i);
		param_get(_params_handles.motor_control_d, &_params.motor_control_d);
		param_get(_params_handles.motor_control_ilim, &_params.motor_control_ilim);

		param_get(_params_handles.motor_control_idleRpm, &_params.motor_control_idleRpm);
		param_get(_params_handles.motor_control_nomRpm, &_params.motor_control_nomRpm);

		param_get(_params_handles.motor_control_upSat, &_params.motor_control_upSat);
		param_get(_params_handles.motor_control_lowSat, &_params.motor_control_lowSat);

		param_get(_params_handles.motor_control_startRate, &_params.motor_control_startRate);
		param_get(_params_handles.motor_control_startRpm, &_params.motor_control_startRpm);

		param_get(_params_handles.motor_control_rate, &_params.motor_control_rate);

		_param_counter = 0;
	 }
	_param_counter++;
 	return OK;
 }

 // We'll still use a poll_all function, but we don't use the old poller anymore. I didn't really like that anyway.
 void
MotorController::poll_all() {

 	bool updated;

 	// Poll things here.

 	// 1: Get the rotor rpm.
 	orb_check(_rotor_rpm_sub, &updated);
 	if(updated) {
 		orb_copy(ORB_ID(rotor_rpm), _rotor_rpm_sub, &_rotor_rpm);
 	}

 	// 2: Get the arming state.
 	orb_check(_actuator_armed_sub, &updated);
 	if(updated) {
 		orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &_actuator_armed);
 	}

 	// 3: Get the rc inputs.
 	orb_check(_rc_channels_sub, &updated);
 	if(updated) {
 		//orb_copy(ORB_ID(rc_channels), _rc_channels_sub, &_rc_channels);
 	}

 }


 void
MotorController::run_controller(float dt)
 {
	 // We do a simple PID controller here. With a few extra conditions imposed.
	 //
	 // 1. If we are not armed the controller is off and the output is zero.
	 // 2. When we arm we must reset the integrators to zero.
	 // 3. We need to perform a "soft" start up of the rotor so as to enable the clutch. In order to do this we need
	 //		to track some switches and states.

	 // Firstly some admin...
	 // 1: Get switch state.
	 if (_rc_channels.channels[8] < -0.5f)
	 {
		 _switch_state = 0;
	 }
	 else if (_rc_channels.channels[8] <= 0.3f && _rc_channels.channels[8] >= -0.5f)
	 {
		 _switch_state = 1;
	 }
	 else if (_rc_channels.channels[8] > 0.3f)
	 {
		 _switch_state = 2;
	 }

	 // 2: Set the state of the kill switch
	 if (!_actuator_armed.armed)
	 {
		 _motor_kill.kill_switch = true;
	 }
	 else if (_actuator_armed.armed && _rc_channels.channels[9] < 0)
	 {
		 _motor_kill.kill_switch = true;
	 }
	 else if (_actuator_armed.armed && _rc_channels.channels[9] >= 0)
	 {
		 _motor_kill.kill_switch = false;
	 }


	 if (_actuator_armed.armed)
	 {
		 // Case 1: Throttle command is off. Do nothing.
		 if (_switch_state == 0)
		{
			// Don't do anything, and keep integrators at zero to stop any winding up or down.
			 _motor_throttle.throttle = 0.0f;
			 _previous_error = 0.0f;
			 _integral_sum = 0.0f;
			 _filtered_rpm_command = 0.0f;
		}

		 // Case 2: Rotor rpm is below cut-off. Then we perform a start procedure.
		 else if (_switch_state == 1 && _rotor_rpm.rpm < _params.motor_control_startRpm)
		 {
			 // We put a start up sequence in here, to get the rotor spinning. For now it is increasing throttle at some
			 // rate. We'll use a parameter to set this.
			 // Once this has finished we switch over to governing with the pid in idle mode.
			 _motor_throttle.throttle = _motor_throttle.throttle + _params.motor_control_startRate;

			 // Set the filtered_rpm_command to the current rpm? Will probably help with command jumps when we enable controller.
			 _filtered_rpm_command = _rotor_rpm.rpm;
		 }

		 // Case 3: We run the governor
		 else if ((_switch_state == 1 || _switch_state == 2) && _rotor_rpm.rpm >= _params.motor_control_startRpm) // I.e. throttle is set to flight mode. Run the controller in full.
		 {
			 // Calculate the filtered rpm command. This is a rate transition from current rpm to target rpm at some rate.
			 if (_switch_state == 1)
			 {
				 // Rate transition command to idle rpm.
				 rate_transition(_filtered_rpm_command, _params.motor_control_idleRpm,dt);
			 }
			 if (_switch_state == 2)
			 {
				 // Rate transition command to flight rpm.
				 rate_transition(_filtered_rpm_command, _params.motor_control_nomRpm,dt);
			 }

			 // Run controller.
			 _motor_throttle.throttle = pid(dt, _filtered_rpm_command, _params.motor_control_ilim, _params.motor_control_upSat, _params.motor_control_lowSat);

		 }
	 }
	 else if (!_actuator_armed.armed)
	 {
		 // Do nothing, and set integrator to zero.
		 _motor_throttle.throttle = 0.0f;
		 _previous_error = 0.0f;
		 _integral_sum = 0.0f;
		 _filtered_rpm_command = 0.0f;
	 }

 }

 float
 MotorController::pid(float dt, float sp, float integratorLimit, float upSat, float lowSat)
 {
	 float error = sp - _rotor_rpm.rpm;						// The error
	 _integral_sum = _integral_sum + error * dt;			// Integral sum
	 float derivative_error = (error - _previous_error);	// Derivative error

	 // Check integrator limits.
	 if (_integral_sum > _params.motor_control_ilim)
	 {
		 _integral_sum = _params.motor_control_ilim;
	 }
	 if (_integral_sum < -_params.motor_control_ilim)
	 {
		 _integral_sum = -_params.motor_control_ilim;
	 }

	 // Check saturation limits.
	 float output = _params.motor_control_p * error + _params.motor_control_i * _integral_sum + _params.motor_control_d * derivative_error;

	 if (output > _params.motor_control_upSat)
	 {
		 output = _params.motor_control_upSat;
	 }
	 if (output < _params.motor_control_lowSat)
	 {
		 output = _params.motor_control_lowSat;
	 }

	 // Return final output and clean up
	 _previous_error = error;
	 return output;
 }


 void
MotorController::rate_transition(float &input, float goal, float dt) {

 	float distance = _params.motor_control_rate * dt;

	if(fabsf(goal - input) < distance) {

		input = goal;

	} else {

		if(goal - input >  0) { // Add to input

			input += distance;
		} else {

			input -= distance;
		}
	}
 }

//Main task handled here.

 void
MotorController::task_main()
 {
	/* get an initial update for all sensor and status data */
 	parameters_update();

 	// Subscriptions
 	_rotor_rpm_sub = orb_subscribe(ORB_ID(rotor_rpm));
 	_actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
 	_rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
 	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* wakeup source */
 	px4_pollfd_struct_t fds[1];
 	fds[0].fd = _vehicle_attitude_sub;
 	fds[0].events = POLLIN;

 	int initialised = 0;

 	while (!_task_should_exit) {
		/* wait for up to 100ms for data */
 		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
 		if (pret == 0) {

 			//Do nothing

 		} else if (pret < 0) { /* this is undesirable but not much we can do */

			//TODO use a performance counter instead of spewing junk to console
 			//warn("poll error %d, %d", pret, errno);

 		} else {

			// Poll all also does our poller.
			poll_all();
			parameters_update();	// We don't want to call this at max rate.

			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.001f) {
				dt = 0.001f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			printf("motor_controller tick: %d \n",(double)dt);

			// If we are initialised we call the run_state_machine function.
			if(initialised > 0) {

				run_controller(dt);

			}
			// It will only go into this else once. This is used to initialise everything.
			else
			{
				// Assign initial values here. Nothing to do for this module.


				initialised++; //We start the state machine up as such.
			}
			assign_and_publish();
 		}
 	}

 	return;
 }

 void
MotorController::assign_and_publish()
 {
	 // Assign. Everything we need has been assigned previously in the code.


	 // Publish - All orb publishing is done in one go here.
	 if (_motor_throttle_pub != nullptr) orb_publish(ORB_ID(motor_throttle), _motor_throttle_pub, &_motor_throttle);
	 else _motor_throttle_pub = orb_advertise(ORB_ID(motor_throttle), &_motor_throttle);

	 // 2: Motor kill switch
	 if (_motor_kill_pub != nullptr) orb_publish(ORB_ID(motor_kill), _motor_kill_pub, &_motor_kill);
     else _motor_kill_pub = orb_advertise(ORB_ID(motor_kill), &_motor_kill);



	 return;
 }

 int
MotorController::start()
 {
 	ASSERT(_control_task == -1);

	/* start the task */
 	_control_task = px4_task_spawn_cmd("motor_controller",
 		SCHED_DEFAULT,
 		SCHED_PRIORITY_MAX,
 		1500,
 		(px4_main_t)&MotorController::task_main_trampoline,
 		nullptr);

 	if (_control_task < 0) {
 		warn("task start failed");
 		return -errno;
 	}

 	return OK;
 }

//Shim for calling task_main from task_create.
 void
MotorController::task_main_trampoline(int argc, char *argv[])
 {
 	motor_controller::g_control->task_main();
 }

 int
 motor_controller_main(int argc, char *argv[])
 {
 	if (argc < 2) {
 		warnx("usage: motor_controller {start|stop|status}");
 		return 1;
 	}

 	if (!strcmp(argv[1], "start")) {

 		if (motor_controller::g_control != nullptr) {
 			warnx("already running");
 			return 1;
 		}

 		motor_controller::g_control = new MotorController;

 		if (motor_controller::g_control == nullptr) {
 			warnx("alloc failed");
 			return 1;
 		}

 		if (OK != motor_controller::g_control->start()) {
 			delete motor_controller::g_control;
 			motor_controller::g_control = nullptr;
 			warnx("start failed");
 			return 1;
 		}

 		return 0;
 	}

 	if (!strcmp(argv[1], "stop")) {
 		if (motor_controller::g_control == nullptr) {
 			warnx("not running");
 			return 1;
 		}

 		delete motor_controller::g_control;
 		motor_controller::g_control = nullptr;
 		return 0;
 	}

 	if (!strcmp(argv[1], "status")) {
 		if (motor_controller::g_control) {
 			warnx("running");
 			return 0;

 		} else {
 			warnx("not running");
 			return 1;
 		}
 	}

 	warnx("unrecognized command");
 	return 1;
 }





