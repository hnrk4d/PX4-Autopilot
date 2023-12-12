/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

#include "tr24da100.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace tr24da100 {

  TR24DA100 *g_dev{nullptr};

  static int start(const char *port, uint8_t rotation) {
    if (g_dev != nullptr) {
      PX4_WARN("already started");
      return -1;
    }

    if (port == nullptr) {
      PX4_ERR("serial port required");
      return -1;
    }

    // Instantiate the driver.
    g_dev = new TR24DA100(port, rotation);

    if (g_dev == nullptr) {
      return -1;
    }

    g_dev->start();

    return 0;
  }

  static int stop() {
    if (g_dev != nullptr) {
      delete g_dev;
      g_dev = nullptr;

    } else {
      return -1;
    }

    return 0;
  }

  static int status() {
    if (g_dev == nullptr) {
      PX4_ERR("driver not running");
      return -1;
    }

    g_dev->print_info();

    return 0;
  }

  static int usage() {
    PRINT_MODULE_DESCRIPTION(
			     R"DESCR_STR(
### Description

Serial bus driver for the TopRadar TR24DA100 radar.

Setup/usage information: http://topradar.net/productview.asp?/1.html

### Examples

Attempt to start driver on a specified serial device.
$ tr24da100 start -d /dev/ttyS1
Stop driver
$ tr24da100 stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("tr24da100", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Serial device", false);
    PRINT_MODULE_USAGE_PARAM_INT('R', distance_sensor_s::ROTATION_DOWNWARD_FACING, 0, distance_sensor_s::ROTATION_DOWNWARD_FACING, "Sensor rotation - downward facing by default", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
    return PX4_OK;
  }

} // namespace tr24da100

extern "C" __EXPORT int tr24da100_main(int argc, char *argv[]) {
  uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
  const char *device_path = nullptr;
  int ch;
  int myoptind = 1;
  const char *myoptarg = nullptr;

 while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'R':
      rotation = (uint8_t)atoi(myoptarg);
      break;

    case 'd':
      device_path = myoptarg;
      break;

    default:
      return tr24da100::usage();
    }
  }

  if (myoptind >= argc) {
    return tr24da100::usage();
  }

  if (!strcmp(argv[myoptind], "start")) {
    return tr24da100::start(device_path, rotation);

  } else if (!strcmp(argv[myoptind], "stop")) {
    return tr24da100::stop();

  } else if (!strcmp(argv[myoptind], "status")) {
    return tr24da100::status();
  }

  return tr24da100::usage();
}
