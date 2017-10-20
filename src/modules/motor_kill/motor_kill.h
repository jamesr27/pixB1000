/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_tracking_control_main.h
 *
 * @brief Helicopter autonomous controller
 *
 * Tracking control as described in Autonomous CLAWS paper. First steps in the outer loops
 * of the model following controller. 
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <uORB/uORB.h>
#include <uORB/topics/motor_kill.h>


 extern "C" __EXPORT int motor_kill_main(int argc, char *argv[]);

 class MotorKill
 {
 public:

	/**
	 * Constructorrc_channels
	 */

	 MotorKill();

	/**
	 * Destructor, also kills task.
	 */

	 ~MotorKill();


	 void 		poll_all();


	/**
	 * Shim for calling task_main from task_create.
	 */
	 static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	 void		task_main();
	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	 int		start();

	private:

		bool 					_task_should_exit;
		int						_control_task;


		int 					_motor_kill_sub;

		motor_kill_s			_motor_kill;
	};
