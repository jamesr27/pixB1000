// Parameters

#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * Motor controller proportional gain
 *
 * @unit rpm*1000/unit out
 * @min 0.00
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_P, 1.0f);	//

/**
 * Motor controller integral gain
 *
 * @unit rpm*1000/unit out
 * @min 0.00
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_I, 0.0f);

/**
 * Motor controller derivative gain
 *
 * @unit rpm*1000/unit out
 * @min 0.00
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_D, 0.0f);

/**
 * Motor controller integrator limit
 *
 * @unit norm
 * @min 0.00
 * @max 0.3
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_ILIM, 0.2f);

/**
 * Motor controller idle rpm
 *
 * @unit rpm
 * @min 300
 * @max 800
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_IDLERPM, 600.0f);

/**
 * Motor controller nominal rpm
 *
 * @unit rpm
 * @min 300
 * @max 1100
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_NOMRPM, 1000.0f);

/**
 * Motor controller upper saturation limit
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_UPSAT, 1.0f);

/**
 * Motor controller lower saturation limit
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_LOWSAT, 0.0f);

/**
 * Motor controller start rate
 *
 * @unit 1/second*1000
 * @min 0
 * @max 1000
 * @decimal 1000
 * @increment 0.1
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_SRATE, 100.0f);

/**
 * Motor controller rpm switchover
 *
 * @unit rpm
 * @min 100
 * @max 1500
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_SRPM, 500.0f);

/**
 * Motor controller sp ramp rate
 *
 * @unit rpm/s
 * @min 10
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_RATE, 50.0f);	//RPM/s


/**
 * Motor controller bypass
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 * @group Default
 */
PARAM_DEFINE_INT32(MOTOR_C_BYPASS, 0);


/**
 * Motor controller idle feed-forward. FF at 0 collective.
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_IFF, 0.15f);	// Actuator output

/**
 * Motor controller flight feed-forward @ 0.5 collective. Use idle feedforward and this in linear fashion to schedule feed forward.
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Default
 */
PARAM_DEFINE_FLOAT(MOTOR_C_FFF, 0.5f);	// Actuator output
