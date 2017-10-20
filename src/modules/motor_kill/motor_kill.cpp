/* James Reeves. Motor (engine) controller for the B1000 helicopter.
This does a few things:
	1. Receives motor armed commands. From the standard uORB.
	2. Receives the throttle command. This comes from the rc at present (channel 9, [8] in zero counting).
	3. Receives the rpm from the rpm sensor (through uORB). (Must add the filing of the rotor rpm mavlink message to mavlink still).
	4. Calculates the throttle output command (PID controller on throttle, or PD, or whatever works best). Published to uORB. We will assign the actuator value in the attitude controller or wherever this is done these days.)
*/

#include "motor_kill.h"
#include "my_serial.h"

namespace motor_kill {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	MotorKill *g_control;

 }

//Constructor
 MotorKill::MotorKill() :
 _task_should_exit(false),
 _control_task(-1),

 	 /* subscriptions */
 _motor_kill_sub(-1)

 {
	memset(&_motor_kill, 0, sizeof(_motor_kill));

 }


//Destructor
MotorKill::~MotorKill()
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

 	motor_kill::g_control = nullptr;

 }


 // We'll still use a poll_all function, but we don't use the old poller anymore. I didn't really like that anyway.
 void
MotorKill::poll_all() {

 	bool updated;

 	// Poll things here.

 	// 1: Get the rotor rpm.
 	orb_check(_motor_kill_sub, &updated);
 	if(updated) {
 		orb_copy(ORB_ID(motor_kill), _motor_kill_sub, &_motor_kill);
 	}


 }


//Main task handled here.

 void
MotorKill::task_main()
 {
 	// Subscriptions
 	_motor_kill_sub = orb_subscribe(ORB_ID(motor_kill));

 	// Serial setup
 	char *uart_name = (char*)"/dev/ttyS6";
	int baudrate = 38400;
	Serial_Port serial_port(uart_name, baudrate);
	serial_port.start();

	char *kill_on = (char*)"\x01\x7F";
	char *kill_off = (char*)"\x00\x7F";

	/* wakeup source */
 	px4_pollfd_struct_t fds[1];
 	fds[0].fd = _motor_kill_sub;
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

			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.001f) {
				dt = 0.001f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}


			// Do our work here.
			if(initialised > 0) {
				// We do the following:
				//	1. Poll for updates.
				//	2. Send serial message depending on contents of _motor_kill.kill_switch

				poll_all();

				if (_motor_kill.kill_switch)
				{
					serial_port.write_bytes(kill_on);
				}
				if (!_motor_kill.kill_switch)
				{
					serial_port.write_bytes(kill_off);
				}
				usleep(10000);

			}
			// It will only go into this else once. This is used to initialise everything.
			else
			{
				// Assign initial values here. Nothing to do for this module.

				initialised++; //We start the state machine up as such.
			}

 		}
 	}

 	return;
 }


 int
MotorKill::start()
 {
 	ASSERT(_control_task == -1);

	/* start the task */
 	_control_task = px4_task_spawn_cmd("motor_kill",
 		SCHED_DEFAULT,
 		SCHED_PRIORITY_MAX,
 		1000,
 		(px4_main_t)&MotorKill::task_main_trampoline,
 		nullptr);

 	if (_control_task < 0) {
 		warn("task start failed");
 		return -errno;
 	}

 	return OK;
 }

//Shim for calling task_main from task_create.
 void
MotorKill::task_main_trampoline(int argc, char *argv[])
 {
 	motor_kill::g_control->task_main();
 }

 int
 motor_kill_main(int argc, char *argv[])
 {
 	if (argc < 2) {
 		warnx("usage: motor_kill {start|stop|status}");
 		return 1;
 	}

 	if (!strcmp(argv[1], "start")) {

 		if (motor_kill::g_control != nullptr) {
 			warnx("already running");
 			return 1;
 		}

 		motor_kill::g_control = new MotorKill;

 		if (motor_kill::g_control == nullptr) {
 			warnx("alloc failed");
 			return 1;
 		}

 		if (OK != motor_kill::g_control->start()) {
 			delete motor_kill::g_control;
 			motor_kill::g_control = nullptr;
 			warnx("start failed");
 			return 1;
 		}

 		return 0;
 	}

 	if (!strcmp(argv[1], "stop")) {
 		if (motor_kill::g_control == nullptr) {
 			warnx("not running");
 			return 1;
 		}

 		delete motor_kill::g_control;
 		motor_kill::g_control = nullptr;
 		return 0;
 	}

 	if (!strcmp(argv[1], "status")) {
 		if (motor_kill::g_control) {
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
