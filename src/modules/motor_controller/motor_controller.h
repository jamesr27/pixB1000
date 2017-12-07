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
#include <uORB/topics/rotor_rpm.h>
#include <uORB/topics/motor_throttle.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/motor_kill.h>
#include <uORB/topics/motor_controller_log.h>


 extern "C" __EXPORT int motor_controller_main(int argc, char *argv[]);

 class MotorController
 {
 public:

	/**
	 * Constructorrc_channels
	 */

	 MotorController();

	/**
	 * Destructor, also kills task.
	 */

	 ~MotorController();


	 int		parameters_update(); 


	 void 		assign_and_publish(float dt);


	 void 		poll_all();

	 void 		run_controller(float dt);

	 float 	    pid(float dt, float sp, float integratorLimit, float upSat, float lowSat);

	 void		rate_transition(float &input, float goal, float dt);

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


		int 					_rotor_rpm_sub;
		int						_actuator_armed_sub;
		int						_rc_channels_sub;
		int						_vehicle_attitude_sub;
		orb_advert_t 			_motor_throttle_pub;
		orb_advert_t			_motor_kill_pub;
		orb_advert_t			_motor_controller_log_pub;

		int 					_param_counter;

		float					_integral_sum;
		float 				    _previous_error;

		int						_switch_state;	//0 - off, 1 - idle, 2- flight.
		float					_filtered_rpm_command; //[0-flightRpm], but passed through rate transition.
															// Used as input to pid controller.
		float 					_throttle_offset;	// [0:1] offset is recorded once we have started.
		bool 					_motor_started;		// False if we haven't started.


		rotor_rpm_s 			_rotor_rpm;
		motor_throttle_s		_motor_throttle;
		actuator_armed_s		_actuator_armed;
		rc_channels_s			_rc_channels;
		motor_kill_s			_motor_kill;
		motor_controller_log_s	_motor_controller_log;


		struct {

			param_t motor_control_p;
			param_t motor_control_i;
			param_t motor_control_d;
			param_t motor_control_ilim;
			param_t motor_control_idleRpm;
			param_t motor_control_nomRpm;
			param_t motor_control_upSat;
			param_t motor_control_lowSat;
			param_t motor_control_startRate;
			param_t motor_control_startRpm;
			param_t motor_control_rate;
			param_t motor_control_bypass;
			param_t motor_control_iff;
			param_t motor_control_fff;

		} _params_handles;

		struct {

			float motor_control_p;
			float motor_control_i;
			float motor_control_d;
			float motor_control_ilim;
			float motor_control_idleRpm;
			float motor_control_nomRpm;
			float motor_control_upSat;
			float motor_control_lowSat;
			float motor_control_startRate;
			float motor_control_startRpm;
			float motor_control_rate;
			int32_t motor_control_bypass;
			float motor_control_iff;
			float motor_control_fff;

		} _params;



	};
