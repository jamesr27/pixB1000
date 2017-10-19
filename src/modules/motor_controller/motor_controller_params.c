// Parameters

#include <px4_config.h>

#include <systemlib/param/param.h>
 
PARAM_DEFINE_FLOAT(MOTOR_C_P, 1.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_I, 0.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_D, 0.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_ILIM, 0.2f);

PARAM_DEFINE_FLOAT(MOTOR_C_IDLERPM, 600.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_NOMRPM, 1000.0f);

PARAM_DEFINE_FLOAT(MOTOR_C_UPSAT, 1.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_LOWSAT, 0.0f);

PARAM_DEFINE_FLOAT(MOTOR_C_SRATE, 0.02f);

PARAM_DEFINE_FLOAT(MOTOR_C_SRPM, 100.0f);
PARAM_DEFINE_FLOAT(MOTOR_C_RATE, 50.0f);	//RPM/s
