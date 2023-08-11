/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

/**
 * Driver for the TopRadar TR24DA100 radar sensor
 */

#pragma once

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

#define TR24DA100_MEASURE_INTERVAL      10_ms
#define TR24DA100_MAX_DISTANCE	        30.0f
#define TR24DA100_MIN_DISTANCE	        0.5f
#define TR24DA100_VERSION	        1
#define BUFFER_MAX_LENGTH               20
#define PACKET_MAX_LENGTH 		10

class TR24DA100 : public px4::ScheduledWorkItem {
public:
  /**
   * Default Constructor
   * @param port The serial port to open for communicating with the sensor.
   * @param rotation The sensor rotation relative to the vehicle body.
   */
  TR24DA100(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
  ~TR24DA100() override;

  void start();
  void print_info();

private:

  /**
   * Reads data from serial UART and places it into a buffer.
   */
  int collect();

  /**
   * Opens and configures the UART serial communications port.
   * @param speed The baudrate (speed) to configure the serial UART port.
   */
  int open_serial_port(const speed_t speed = B115200);

  void Run() override;
  void stop();

  PX4Rangefinder _px4_rangefinder;

  char _port[20] {};
  int _file_descriptor{-1};

  perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
  perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

  const uint8_t sFrameHeader[2] = {0xa5, 0x5a};
  uint8_t _buffer[BUFFER_MAX_LENGTH];
  uint8_t _packet[PACKET_MAX_LENGTH];

  int _packetIndex;
  int _num;
  uint8_t _lastReadByte;
  float _dist_m;
  //int _counted_packets;
  hrt_abstime _timestamp_last_sample;
  unsigned int _reopen_attempts;
};
