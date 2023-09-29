/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

#include "flktr2l.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
//#include <uORB/topics/adc_report.h>

#include <parameters/param.h>

int Flktr2L::print_status() {
  PX4_INFO("Running, failsafe status : %d, scale : %lu", _failsafe, _package._scale);

  return PX4_OK;
}

int Flktr2L::custom_command(int argc, char *argv[]) {

  if (!is_running()) {
    print_usage("not running");
    return PX4_ERROR;
  }

  // additional custom commands can be handled like this:
  /*
  if (!strcmp(argv[0], "xyz")) {
    get_instance()->xyz();
    return PX4_OK;
    }*/

  return print_usage("unknown command");
}

int Flktr2L::task_spawn(int argc, char *argv[]) {
  _task_id = px4_task_spawn_cmd("flktr2l",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				2048,
				(px4_main_t)&run_trampoline,
				(char *const *)argv);

  if (_task_id < 0) {
    _task_id = -1;
    return -errno;
  }

  return PX4_OK;
}

Flktr2L *Flktr2L::instantiate(int argc, char *argv[]) {
  bool error_flag = false;

  int myoptind = 1;
  int ch;
  const char *myoptarg = nullptr;
  char port[20] = "/dev/ttyS1";
  speed_t speed = B115200;

  // parse CLI arguments
  while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case '?':
      error_flag = true;
      break;

    case 'd':      
      strncpy(port, myoptarg, sizeof(port));
      port[sizeof(port)-1] = 0x0;
      break;
      
    case 'b':
      switch (atoi(myoptarg)) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	  default:
	    PX4_WARN("unknown baud rate");
	    error_flag = true;
	}

      break;

    default:
      PX4_WARN("unrecognized flag");
      error_flag = true;
      break;
    }
  }
  
  if (error_flag) {
    return nullptr;
  }

  Flktr2L *instance = new Flktr2L(port, speed);

  if (instance == nullptr) {
    PX4_ERR("alloc failed");
  }

  return instance;
}

Flktr2L::Flktr2L(const char *port, const speed_t speed)
  : ModuleParams(nullptr) {
  strncpy(_port, port, sizeof(_port));
  _port[sizeof(_port)-1]=0x0;
  _speed = speed;
  _timestamp_last_sample = hrt_absolute_time(); //reset time counter
}

int Flktr2L::open_serial_port(const char *port, const speed_t speed) {  
  // File descriptor initialized?
  if (_file_descriptor > 0) {
    PX4_DEBUG("serial port already open");
    return PX4_OK;
  }

  // Configure port flags for read/write, non-controlling, non-blocking.
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  // Open the serial port.
  _file_descriptor = ::open(port, flags);

  if (_file_descriptor < 0) {
    PX4_ERR("open failed (%i)", errno);
    return PX4_ERROR;
  }

  if (!isatty(_file_descriptor)) {
    PX4_WARN("not a serial device");
    return PX4_ERROR;
  }

  termios uart_config{};

  // Store the current port configuration. attributes.
  if (tcgetattr(_file_descriptor, &uart_config)) {
    PX4_ERR("Unable to get termios from %s.", port);
    ::close(_file_descriptor);
    _file_descriptor = -1;
    return PX4_ERROR;
  }

  // Clear: data bit size, two stop bits, parity, hardware flow control.
  uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

  // Set: 8 data bits, enable receiver, ignore modem status lines.
  uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

  // Clear: echo, echo new line, canonical input and extended input.
  uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

  // Clear ONLCR flag (which appends a CR for every LF).
  uart_config.c_oflag &= ~ONLCR;

  // Set the input baud rate in the uart_config struct.
  int termios_state = cfsetispeed(&uart_config, speed);

  if (termios_state < 0) {
    PX4_ERR("CFG: %d ISPD", termios_state);
    ::close(_file_descriptor);
    return PX4_ERROR;
  }

  // Set the output baud rate in the uart_config struct.
  termios_state = cfsetospeed(&uart_config, speed);

  if (termios_state < 0) {
    PX4_ERR("CFG: %d OSPD", termios_state);
    ::close(_file_descriptor);
    return PX4_ERROR;
  }

  // Apply the modified port attributes.
  termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

  if (termios_state < 0) {
    PX4_ERR("baud %d ATTR", termios_state);
    ::close(_file_descriptor);
    return PX4_ERROR;
  }

  // Flush the hardware buffers.
  tcflush(_file_descriptor, TCIOFLUSH);

  //PX4_INFO("successfully opened UART port %s", port);

  return PX4_OK;
}

void Flktr2L::_shiftAndAdd(uint8_t oneByte) {
  uint8_t *pkg = (uint8_t*)&_package;
  for(unsigned int i=1; i<sizeof(_package); ++i) {
    pkg[i-1] = pkg[i];
  }
  pkg[sizeof(_package)-1] = oneByte;
}

void Flktr2L::run() {
  struct vehicle_status_s vehicle_status;
  //struct adc_report_s adc_report;

  int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  //int adc_report_sub = orb_subscribe(ORB_ID(adc_report));

  px4_pollfd_struct_t fds[1];
  fds[0].fd = vehicle_status_sub;
  fds[0].events = POLLIN;
  //fds[1].fd = adc_report_sub;
  //fds[1].events = POLLIN;

  // initialize parameters
  parameters_update(true);

  //hrt_abstime _timestamp_last_adc_report = hrt_absolute_time();
  
  while (!should_exit()) {
    // wait for up to 1000ms for data
    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
    
    if (pret == 0) {
      // Timeout: let the loop run anyway, don't do `continue` here
    } else if (pret < 0) {
      // this is undesirable but not much we can do
      PX4_ERR("poll error %d, %d", pret, errno);
      px4_usleep(50000);
      continue;
    } else {
      if (fds[0].revents & POLLIN) {
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
	
	if(vehicle_status.failsafe != _failsafe) {
	  //failsafe status changed
	  _failsafe = vehicle_status.failsafe;
	  PX4_INFO("failsafe status changed to %d", _failsafe);
	  if(_failsafe) {
	    vehicle_command_s vcmd = {}; // all 0 is fine
	    vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	    vcmd.param1=-1; //all off
	    vcmd.param2=-1;
	    vcmd.param3=-1;
	    vcmd.param4=-1;
	    vcmd.param5=-1;
	    vcmd.param6=-1;
	    vcmd.timestamp = hrt_absolute_time();
	    _vehicle_command_pub.publish(vcmd);
	  }
	}
      }
      /*
      if (fds[1].revents & POLLIN) {
	if (hrt_elapsed_time(&_timestamp_last_adc_report) > 900000) {
	  _timestamp_last_adc_report = hrt_absolute_time(); //reset time counter
	  orb_copy(ORB_ID(adc_report), adc_report_sub, &adc_report);

	  _msg.id=0;
	  _msg.timestamp = adc_report.timestamp;
	  _msg.data_1 = adc_report.raw_data[0];
	  _msg.data_2 = adc_report.raw_data[1];
	  _tool_status_pub.publish(_msg);
	}
      }
      */

      parameters_update();
    }

    //UART communication
    open_serial_port(_port, _speed);
    bool res = false;
    int bytes_read = 0;
    while((bytes_read = ::read(_file_descriptor, &_buffer[0], sizeof(_buffer))) >0) {	
      for(int i=0; i<bytes_read; i++) {
	_shiftAndAdd(_buffer[i]);
	if(_package._header[0] == sReferenceHeader[0] && _package._header[1] == sReferenceHeader[1]) {
	  // we found a valid frame
	  res = true;
	  _msg.id=0;
	  _msg.timestamp = hrt_absolute_time();
	  _msg.data_1 = _package._scale;
	  _msg.data_2 = 0;
	  //PX4_INFO("%lu", _msg.data_1);
	}
      }
    }

    if(res) {
      _tool_status_pub.publish(_msg); //last value only
      _timestamp_last_sample = hrt_absolute_time(); //remember when we read the last valid package
    }
    else {
      //when did we read the last valid package
      if (hrt_elapsed_time(&_timestamp_last_sample) > 10000000) {
	_timestamp_last_sample = hrt_absolute_time(); //reset time counter
	//let's try to reopen the port
	::close(_file_descriptor);
	_file_descriptor = -1;
	open_serial_port(_port, _speed);
      }
      }
    
    px4_usleep(100000);
  }

  orb_unsubscribe(vehicle_status_sub);
  //orb_unsubscribe(adc_report_sub);
  close(_file_descriptor);
}

/*
void Flktr2L::adc() {
  PX4_INFO("ADC: %ld %ld",
	_msg.data_1,
	_msg.data_2
  );
}
*/

void Flktr2L::parameters_update(bool force) {
  // check for parameter updates
  if (_parameter_update_sub.updated() || force) {
    // clear update
    parameter_update_s update;
    _parameter_update_sub.copy(&update);
    
    // update parameters from storage
    updateParams();
  }
}

int Flktr2L::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }
  
  PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
The module flktr2l supports start/stop/status functionality and runs in the background.

### Implementation
The module flktr2l is listening to the vehicle and tool status. If a failsafe status is triggered, the actuators will be turned off. Depending on the tool it collects tool depending information.

### Examples
CLI usage example:
$ flktr2l start

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("flktr2l", "system");
  PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start module");
  PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "/dev/ttyS[0-9]", "Serial device", false);
  PRINT_MODULE_USAGE_PARAM_STRING('b', "115200", "9600, 19200, 38400, 57600, 115200, 230400", "Baud rate", false);

  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return PX4_OK;
}

int flktr2l_main(int argc, char *argv[]) {
  return Flktr2L::main(argc, argv);
}
