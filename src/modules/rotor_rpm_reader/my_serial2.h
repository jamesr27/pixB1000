// James hacks this to work for reading mavlink serial data on serial port 2. This only contains the rotor rpm data.

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#ifndef __PX4_POSIX
#include <termios.h>
#endif  
// POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <nuttx/serial/serial.h>
#include <nuttx/config.h>

#include <v2.0/B1000/mavlink.h>
 

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// Status flags
#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

//class Serial_Port;



// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * guards any port operation with a pthread mutex.
 */
class Serial_Port2
{

public:

	Serial_Port2();
	Serial_Port2(char *&uart_name_, int &baudrate_);
	void initialize_defaults();
	~Serial_Port2();

	bool debug;
	char *uart_name;
	int  baudrate;
	int  status;

	int read_message(mavlink_message_t &message);
//	int	write_message(mavlink_message_t &message);


	void open_serial();
	void close_serial();

	void start(); //Doesn't do anything but start open and close
	void stop(); //For convenience I suppose

private:

	int  fd;
	mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	int  _open_port(const char* port);
	bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	int  _read_port(uint8_t &cp);
	void _write_port(char *buf, unsigned &len);

};



#endif // SERIAL_PORT_H_


