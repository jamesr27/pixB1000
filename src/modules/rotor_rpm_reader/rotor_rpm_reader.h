// James Reeves.
// Rotor rpm serial message handler

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
#include <uORB/topics/rotor_rpm.h>


 extern "C" __EXPORT int rotor_rpm_reader_main(int argc, char *argv[]);

 class RotorRpmReader
 {
 public:

	/**
	 * Constructorrc_channels
	 */

	 RotorRpmReader();

	/**
	 * Destructor, also kills task.
	 */

	 ~RotorRpmReader();

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

		orb_advert_t			_rotor_rpm_pub;
		rotor_rpm_s 			_rotor_rpm;

	};
