/* James Reeves. Motor (engine) controller for the B1000 helicopter.
 * This handles serial data inbound on port 2, and puts it into the rotor_rpm uorb.
 * The inbound data are mavlink messages.
*/

#include "rotor_rpm_reader.h"

#include "my_serial2.h"

namespace rotor_rpm_reader {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	RotorRpmReader *g_control;

 }

//Constructor
RotorRpmReader::RotorRpmReader() :
 _task_should_exit(false),
 _control_task(-1),

 // Publications
_rotor_rpm_pub(nullptr)

 {
 	memset(&_rotor_rpm, 0, sizeof(_rotor_rpm));
 }


//Destructor
RotorRpmReader::~RotorRpmReader()
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

 	rotor_rpm_reader::g_control = nullptr;

 }


//Main task handled here.

 void
 RotorRpmReader::task_main()
 {
 	// Subscriptions

	// Publications


 	// Serial setup
 	char *uart_name = (char*)"/dev/ttyS2";
	int baudrate = 57600;
	Serial_Port2 serial_port(uart_name, baudrate);
	serial_port.start();

	// House keeping
	bool success;               // receive success flag
	mavlink_rotorrpm_t _rotor_rpm_m;

 	while (!_task_should_exit) {

 		// Read mesage from serial port
		mavlink_message_t message;
		success = serial_port.read_message(message);


 		// Assign and publish if there is an updated message.
 		if (success){
 			// Handle message
 			switch (message.msgid)
 			{
 			case MAVLINK_MSG_ID_ROTORRPM:
 			//	printf("Identified rotor rpm message.\n");
				mavlink_msg_rotorrpm_decode(&message, &(_rotor_rpm_m));
 				break;

 			default:
			//	printf("Some other message\n");
 				break;
 			}

			// Assign
 			_rotor_rpm.rpm = _rotor_rpm_m.rpm;

			// Publish
			if (_rotor_rpm_pub != nullptr) orb_publish(ORB_ID(rotor_rpm), _rotor_rpm_pub, &_rotor_rpm);
			else _rotor_rpm_pub = orb_advertise(ORB_ID(rotor_rpm), &_rotor_rpm);

			success = false;
 		}


 		usleep(2000);
 	}

 	return;
 }


 int
 RotorRpmReader::start()
 {
 	ASSERT(_control_task == -1);

	/* start the task */
 	_control_task = px4_task_spawn_cmd("rotor_rpm_reader",
 		SCHED_DEFAULT,
 		SCHED_PRIORITY_MAX-40,
 		1500,
 		(px4_main_t)&RotorRpmReader::task_main_trampoline,
 		nullptr);

 	if (_control_task < 0) {
 		warn("task start failed");
 		return -errno;
 	}

 	return OK;
 }

//Shim for calling task_main from task_create.
 void
 RotorRpmReader::task_main_trampoline(int argc, char *argv[])
 {
	 rotor_rpm_reader::g_control->task_main();
 }

 int
 rotor_rpm_reader_main(int argc, char *argv[])
 {
 	if (argc < 2) {
 		warnx("usage: rotor_rpm_reader {start|stop|status}");
 		return 1;
 	}

 	if (!strcmp(argv[1], "start")) {

 		if (rotor_rpm_reader::g_control != nullptr) {
 			warnx("already running");
 			return 1;
 		}

 		rotor_rpm_reader::g_control = new RotorRpmReader;

 		if (rotor_rpm_reader::g_control == nullptr) {
 			warnx("alloc failed");
 			return 1;
 		}

 		if (OK != rotor_rpm_reader::g_control->start()) {
 			delete rotor_rpm_reader::g_control;
 			rotor_rpm_reader::g_control = nullptr;
 			warnx("start failed");
 			return 1;
 		}

 		return 0;
 	}

 	if (!strcmp(argv[1], "stop")) {
 		if (rotor_rpm_reader::g_control == nullptr) {
 			warnx("not running");
 			return 1;
 		}

 		delete rotor_rpm_reader::g_control;
 		rotor_rpm_reader::g_control = nullptr;
 		return 0;
 	}

 	if (!strcmp(argv[1], "status")) {
 		if (rotor_rpm_reader::g_control) {
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
