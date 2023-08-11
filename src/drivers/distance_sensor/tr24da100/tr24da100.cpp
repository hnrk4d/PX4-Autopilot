/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

#include "tr24da100.hpp"

#include <lib/drivers/device/Device.hpp>

TR24DA100::TR24DA100(const char *port, uint8_t rotation) :
  ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
  _px4_rangefinder(0, rotation) {
  /* store port name */
  strncpy(_port, port, sizeof(_port) - 1);

  /* enforce null termination */
  _port[sizeof(_port) - 1] = '\0';

  //PX4_INFO("TR24DA100::TR24DA100 %s", _port);
  
  device::Device::DeviceId device_id;
  device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

  uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

  if (bus_num < 10) {
    device_id.devid_s.bus = bus_num;
  }

  _px4_rangefinder.set_device_id(device_id.devid);
  _px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TR24DA100);
  _px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);

  _px4_rangefinder.set_min_distance(TR24DA100_MIN_DISTANCE);
  _px4_rangefinder.set_max_distance(TR24DA100_MAX_DISTANCE);

  _dist_m=-1; //a negative value will indicate a disconnected sensor
  _packetIndex=0;
  _num=0;
  _lastReadByte=0;
  _timestamp_last_sample=-1;
  _reopen_attempts=0;
  //_counted_packets=0;
}

TR24DA100::~TR24DA100() {
  //PX4_INFO("TR24DA100::~TR24DA100");
  stop();

  perf_free(_sample_perf);
  perf_free(_comms_errors);
}

int TR24DA100::collect() {
  perf_begin(_sample_perf);

  int bytes_read = ::read(_file_descriptor, &_buffer[0], sizeof(_buffer));
	
  bool res=false;
  
  if (bytes_read > 0) {
    //_counted_packets+=bytes_read;
    for(int i=0; i<bytes_read; i++) {
      uint8_t inByte = _buffer[i];
      if(_lastReadByte == sFrameHeader[0] && inByte == sFrameHeader[1]) {
	//we detected a new frame
	if(_packetIndex >= 4) {
	  //evaluate non-empty frame
	  _dist_m = ((float)(((int)_packet[2])*256+(int)_packet[3]))/1000.0f;
	  res=true;
	}
	_packetIndex=0; //reset counter and packet+
	_packet[_packetIndex++]=_lastReadByte;
	_packet[_packetIndex++]=inByte;
      }
      else if(_packetIndex < PACKET_MAX_LENGTH) {
	_packet[_packetIndex++]=inByte;
      }
      _lastReadByte=inByte;
    }
  }

  if(res) {
    _timestamp_last_sample = hrt_absolute_time(); //remember when we read the last valid package
    _px4_rangefinder.update(_timestamp_last_sample, _dist_m);
  }
  else {
    //when did we read the last valid package
    if (hrt_elapsed_time(&_timestamp_last_sample) > 1000000) {
      _timestamp_last_sample = hrt_absolute_time(); //reset time counter
      //let's try to reopen the port
      ::close(_file_descriptor);
      _file_descriptor = -1;
      open_serial_port();
      _reopen_attempts++; //inc attempts for reporting
    }
  }

  perf_end(_sample_perf);
  return PX4_OK;
}

int TR24DA100::open_serial_port(const speed_t speed) {
  // File descriptor initialized?
  if (_file_descriptor > 0) {
    PX4_DEBUG("serial port already open");
    return PX4_OK;
  }

  // Configure port flags for read/write, non-controlling, non-blocking.
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  // Open the serial port.
  _file_descriptor = ::open(_port, flags);

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
    PX4_ERR("Unable to get termios from %s.", _port);
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

  //PX4_INFO("successfully opened UART port %s", _port);

  return PX4_OK;
}

void TR24DA100::Run() {
  //ensure the serial port is open
  open_serial_port();

  collect();
/*
  const uint8_t sFrame[] = {0xa5, 0x5a, 0x0d, 0x56, 0x34, 0xd4};
  int num_bytes = ::write(_file_descriptor, sFrame, sizeof(sFrame));
  _counted_packets+=num_bytes;
*/
  //tcflush(_file_descriptor, TCIFLUSH);
}

void TR24DA100::start() {
  _timestamp_last_sample = hrt_absolute_time(); //reset time counter
  // Schedule the driver at regular intervals.
  ScheduleOnInterval(TR24DA100_MEASURE_INTERVAL, TR24DA100_MEASURE_INTERVAL);
}

void TR24DA100::stop() {
  // Clear the work queue schedule.
  ScheduleClear();

  // Ensure the serial port is closed.
  ::close(_file_descriptor);
  _file_descriptor = -1;
}

void TR24DA100::print_info() {
  perf_print_counter(_sample_perf);
  perf_print_counter(_comms_errors);

  PX4_INFO("cur dist = %.2f m", ((double)_dist_m));
  PX4_INFO("port reopen attempts = %u", (_reopen_attempts));
  //PX4_INFO("packets = %d", _counted_packets);
}
