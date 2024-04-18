/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

#include "tooldrv.h"

#include <math.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/adc_report.h>

#include <parameters/param.h>

int ToolDrv::print_status() {
  PX4_INFO("Running");
  return PX4_OK;
}

int ToolDrv::custom_command(int argc, char *argv[]) {

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

int ToolDrv::task_spawn(int argc, char *argv[]) {
  _task_id = px4_task_spawn_cmd("tooldrv",
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

ToolDrv *ToolDrv::instantiate(int argc, char *argv[]) {
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

  ToolDrv *instance = new ToolDrv(port, speed);

  if (instance == nullptr) {
    PX4_ERR("alloc failed");
  }

  return instance;
}

ToolDrv::ToolDrv(const char *port, const speed_t speed)
  : ModuleParams(nullptr),
    _px4_rangefinder_forward(0, distance_sensor_s::ROTATION_FORWARD_FACING),
    _px4_rangefinder_downward(1, distance_sensor_s::ROTATION_DOWNWARD_FACING) {
  strncpy(_port, port, sizeof(_port));
  _port[sizeof(_port)-1]=0x0;
  _speed = speed;
  _px4_rangefinder_forward.set_device_type(DRV_DIST_DEVTYPE_TR24DA100);
  _px4_rangefinder_forward.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
  _px4_rangefinder_forward.set_min_distance(0.15f);
  _px4_rangefinder_forward.set_max_distance(50.0f);

  _px4_rangefinder_downward.set_device_type(DRV_DIST_DEVTYPE_TR24DA100);
  _px4_rangefinder_downward.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
  _px4_rangefinder_downward.set_min_distance(0.15f);
  _px4_rangefinder_downward.set_max_distance(50.0f);

  _timestamp_last_sample = hrt_absolute_time(); //reset time counter
  _px42teensy.reset();
  _timestamp_last_write = hrt_absolute_time();
  _timestamp_last_speed = hrt_absolute_time();
}

int ToolDrv::open_serial_port(const char *port, const speed_t speed) {  
  // File descriptor initialized?
  if (_file_descriptor > 0) {
    PX4_DEBUG("serial port already open");
    return PX4_OK;
  }

  // Configure port flags for read/write, non-controlling, non-blocking.
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);

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
  ::tcflush(_file_descriptor, TCIOFLUSH);

  //PX4_INFO("successfully opened UART port %s", port);

  return PX4_OK;
}

void ToolDrv::_shiftAndAdd(uint8_t oneByte) {
  uint8_t *pkg = (uint8_t*)&_teensy2px4;
  memmove(pkg, pkg+1, sizeof(_teensy2px4)-1);
  pkg[sizeof(_teensy2px4)-1] = oneByte;
}

void ToolDrv::run() {  
  struct vehicle_status_s vehicle_status, vehicle_status_dup;
  struct vehicle_command_s vehicle_command;
  struct vehicle_local_position_s vehicle_local_position;
  //struct adc_report_s adc_report;

  int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  int vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
  int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
  //int adc_report_sub = orb_subscribe(ORB_ID(adc_report));

  orb_set_interval(vehicle_local_position_sub, 250);
  px4_pollfd_struct_t fds[] =
    {
      {.fd = vehicle_status_sub, .events = POLLIN},
      {.fd = vehicle_command_sub, .events = POLLIN},
      {.fd = vehicle_local_position_sub, .events = POLLIN}
      //{.fd = adc_report_sub, .events = POLLIN}
    };

  // initialize parameters
  parameters_update(true);

  //hrt_abstime _timestamp_last_adc_report = hrt_absolute_time();
  
  while (!should_exit()) {
    // wait for up to 100 ms for data
    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 100);
    _px42teensy.reset();
    if (pret == 0) {
      // Timeout: let the loop run anyway, don't do `continue` here
    } else if (pret < 0) {
      // this is undesirable but not much we can do
      PX4_ERR("poll error %d, %d", pret, errno);
      px4_usleep(50000);
      continue;
    } else {
      if (fds[0].revents & POLLIN) {
	//vehicle status changed
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
	if(vehicle_status_dup.arming_state != vehicle_status.arming_state ||
	   vehicle_status_dup.nav_state != vehicle_status.nav_state ||
	   vehicle_status_dup.failsafe != vehicle_status.failsafe) {
	  //if we change to disarmed, we turn off the actuators
	  if(vehicle_status_dup.arming_state != vehicle_status.arming_state &&
	     vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED
	     ) {
	    vehicle_command_s vcmd = {}; // init to 0
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
	  orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status_dup);
	  _px42teensy._mod |= PckgPX42Teensy::STATUS;
	  _px42teensy._arming_state = vehicle_status.arming_state;
	  _px42teensy._nav_state = vehicle_status.nav_state;
	  _px42teensy._failsafe = vehicle_status.failsafe;
	  PX4_INFO("status change: %d/%d/%d", _px42teensy._arming_state, _px42teensy._nav_state, _px42teensy._failsafe);
	}
      }
      if (fds[1].revents & POLLIN) {
	//vehicle command -> target speed
	orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &vehicle_command);
	if(vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {
	  if(vehicle_command.param2 >= 0) {
	    _px42teensy._target_speed = vehicle_command.param2;
	    _px42teensy._mod |= PckgPX42Teensy::TARGET_SPEED;
	    PX4_INFO("mission speed change: %.2f", (double)_px42teensy._target_speed);
	  }
	}
	if(vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR) {
	  _px42teensy._aux[0] = vehicle_command.param1;
	  _px42teensy._aux[1] = vehicle_command.param2;
	  _px42teensy._aux[2] = vehicle_command.param3;
	  _px42teensy._aux[3] = vehicle_command.param4;
	  _px42teensy._aux[4] = vehicle_command.param5;
	  _px42teensy._aux[5] = vehicle_command.param6;
	  _px42teensy._mod |= PckgPX42Teensy::AUX1 | PckgPX42Teensy::AUX2 | PckgPX42Teensy::AUX3 | PckgPX42Teensy::AUX4 | PckgPX42Teensy::AUX5 | PckgPX42Teensy::AUX6;
	  _px42teensy._test = false;
 	  PX4_INFO("actuators: %.2f %.2f %.2f %.2f %.2f %.2f",
		   (double)_px42teensy._aux[0],
		   (double)_px42teensy._aux[1],
		   (double)_px42teensy._aux[2],
		   (double)_px42teensy._aux[3],
		   (double)_px42teensy._aux[4],
		   (double)_px42teensy._aux[5]);
	}
	if(vehicle_command.command == vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST) {
	  int aux = ::round(vehicle_command.param5 - 1301.0); //see actuator functions in https://docs.px4.io/main/en/advanced_config/parameter_reference.html
	  if(aux >= 0 && aux < 6) {
	    for(auto &x : _px42teensy._aux) x = -1;
	    _px42teensy._aux[aux] = isnanf(vehicle_command.param1)?-1:vehicle_command.param1;
	    _px42teensy._test = true;
	    switch(aux) {
	    case 0: _px42teensy._mod |= PckgPX42Teensy::AUX1; break;
	    case 1: _px42teensy._mod |= PckgPX42Teensy::AUX2; break;
	    case 2: _px42teensy._mod |= PckgPX42Teensy::AUX3; break;
	    case 3: _px42teensy._mod |= PckgPX42Teensy::AUX4; break;
	    case 4: _px42teensy._mod |= PckgPX42Teensy::AUX5; break;
	    case 5: _px42teensy._mod |= PckgPX42Teensy::AUX6; break;
	    }
	    _px42teensy._mod |= PckgPX42Teensy::TEST;
	    PX4_INFO("actuator test: %.2f %.2f %.2f %.2f %.2f %.2f",
		     (double)_px42teensy._aux[0],
		     (double)_px42teensy._aux[1],
		     (double)_px42teensy._aux[2],
		     (double)_px42teensy._aux[3],
		     (double)_px42teensy._aux[4],
		     (double)_px42teensy._aux[5]);
	  }
	}
      }
      if (fds[2].revents & POLLIN) {
	//actual speed
	if(hrt_elapsed_time(&_timestamp_last_speed) > 300000) {
	  orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position);
	  _px42teensy._actual_speed = sqrt(vehicle_local_position.vx*vehicle_local_position.vx + vehicle_local_position.vy*vehicle_local_position.vy);
	  _px42teensy._mod |= PckgPX42Teensy::ACTUAL_SPEED;
	   _timestamp_last_speed = hrt_absolute_time();
	   //PX4_INFO("vehicle speed: %.2f %.2f", (double)vehicle_local_position.vx, (double)vehicle_local_position.vy);
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

    //read
    bool res = false;
    int bytes_read = 0;
    while((bytes_read = ::read(_file_descriptor, &_buffer[0], sizeof(_buffer))) >0) {	
      for(int i=0; i<bytes_read; i++) {
	_shiftAndAdd(_buffer[i]);
	if(_teensy2px4._header[0] == sReferenceHeader[0] && _teensy2px4._header[1] == sReferenceHeader[1] && _teensy2px4._size == sizeof(_teensy2px4)) {
	  uint16_t crc = crc16((char*)&_teensy2px4._size, sizeof(_teensy2px4)-3*sizeof(uint16_t));
          if(crc == _teensy2px4._crc) {
	    // we found a valid frame
	    res = true;
	    if(_teensy2px4._mod & PckgTeensy2PX4::SCALE || _teensy2px4._mod & PckgTeensy2PX4::PWM) {
	      if(_teensy2px4._mod & PckgTeensy2PX4::SCALE) {
		_msg_tool_status.id |= 0x01;
		_msg_tool_status.timestamp = hrt_absolute_time();
		_msg_tool_status.weight = _teensy2px4._scale;
		_msg_tool_status.data   = 0;
	      }
	      if(_teensy2px4._mod & PckgTeensy2PX4::PWM) {
		_msg_tool_status.id |= 0x02;
		_msg_tool_status.pwm_0 = _teensy2px4._pwm[0];
		_msg_tool_status.pwm_1 = _teensy2px4._pwm[1];
		_msg_tool_status.pwm_2 = _teensy2px4._pwm[2];
		_msg_tool_status.pwm_3 = _teensy2px4._pwm[3];
		_msg_tool_status.pwm_4 = _teensy2px4._pwm[4];
		_msg_tool_status.pwm_5 = _teensy2px4._pwm[5];
	      }
	      _tool_status_pub.publish(_msg_tool_status);
	    }
	    if(_teensy2px4._mod & PckgTeensy2PX4::DOWNWARD) {
	      _px4_rangefinder_downward.update(hrt_absolute_time(), _teensy2px4._downward_dist);	    
	    }
	    if(_teensy2px4._mod & PckgTeensy2PX4::FORWARD) {
	      _px4_rangefinder_forward.update(hrt_absolute_time(), _teensy2px4._forward_dist);	    
	    }
	    /*
	    PX4_INFO("teensy2px4: %d - %lu %.2f %.2f [%d %d %d %d %d %d]", (int)_teensy2px4._mod,
		     _teensy2px4._scale, (double)_teensy2px4._downward_dist, (double)_teensy2px4._forward_dist,
		     _teensy2px4._pwm[0], _teensy2px4._pwm[1], _teensy2px4._pwm[2],
		     _teensy2px4._pwm[3], _teensy2px4._pwm[4], _teensy2px4._pwm[5]
		     );
	    */
	    _teensy2px4._mod = 0;
	  }
	}
      }
    }

    if(res) {
      _timestamp_last_sample = hrt_absolute_time(); //remember when we read the last valid package
    }
    else {
      //when did we read the last valid package?
      if (hrt_elapsed_time(&_timestamp_last_sample) > 10000000) {
	_timestamp_last_sample = hrt_absolute_time(); //reset time counter
	//let's try to reopen the port
	::close(_file_descriptor);
	_file_descriptor = -1;
	open_serial_port(_port, _speed);
      }
    }

    px4_usleep(10000); //avoid timeing issues on UART between read and write

    //write
    if (_px42teensy._mod || (hrt_elapsed_time(&_timestamp_last_write) > 5000000)) {
      _px42teensy._crc = crc16((char*)&_px42teensy._size, sizeof(_px42teensy)-3*sizeof(uint16_t));
      _timestamp_last_write = hrt_absolute_time();
      //int bytes_written =
        ::write(_file_descriptor, &_px42teensy, sizeof(_px42teensy));
      //::tcdrain(_file_descriptor);
      //::tcflush(_file_descriptor, TCOFLUSH);
      /*PX4_INFO("bytes written: %d: %.2f %.2f %.2f %.2f %.2f %.2f",
	       bytes_written,
	       (double)_px42teensy._aux[0],
	       (double)_px42teensy._aux[1],
	       (double)_px42teensy._aux[2],
	       (double)_px42teensy._aux[3],
	       (double)_px42teensy._aux[4],
	       (double)_px42teensy._aux[5]);*/
    }    

    px4_usleep(20000);
  }

  orb_unsubscribe(vehicle_status_sub);
  orb_unsubscribe(vehicle_command_sub);
  orb_unsubscribe(vehicle_local_position_sub);
  //orb_unsubscribe(adc_report_sub);
  close(_file_descriptor);
}

uint16_t ToolDrv::crc16(char *data_p, uint16_t length) {
  unsigned int POLY = 0x8408;
  unsigned char i;
  unsigned int data;
  unsigned int crc = 0xffff;

  if (length == 0)
    return (~crc);

  do {
    for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001))
	crc = (crc >> 1) ^ POLY;
      else  crc >>= 1;
    }
  } while (--length);
  crc = ~crc;
  data = crc;
  crc = (crc << 8) | ((data >> 8) & 0xff);

  return (crc);
}

/*
void ToolDrv::adc() {
  PX4_INFO("ADC: %ld %ld",
	_msg.data_1,
	_msg.data_2
  );
}
*/

void ToolDrv::parameters_update(bool force) {
  // check for parameter updates
  if (_parameter_update_sub.updated() || force) {
    // clear update
    parameter_update_s update;
    _parameter_update_sub.copy(&update);
    
    // update parameters from storage
    updateParams();
  }
}

int ToolDrv::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }
  
  PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
The module tooldrv supports start/stop/status functionality and runs in the background.

### Implementation
The module tooldrv is listening to the vehicle and tool status. If a failsafe status is triggered, the actuators will be turned off. Depending on the tool it collects tool depending information.

### Examples
CLI usage example:
$ tooldrv start

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("tooldrv", "system");
  PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start module");
  PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "/dev/ttyS[0-9]", "Serial device", false);
  PRINT_MODULE_USAGE_PARAM_STRING('b', "115200", "9600, 19200, 38400, 57600, 115200, 230400", "Baud rate", false);

  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return PX4_OK;
}

int tooldrv_main(int argc, char *argv[]) {
  return ToolDrv::main(argc, argv);
}
