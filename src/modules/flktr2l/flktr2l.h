/****************************************************************************
 *
 *   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <termios.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/tool_status.h>

using namespace time_literals;

extern "C" __EXPORT int flktr2l_main(int argc, char *argv[]);

class Flktr2L : public ModuleBase<Flktr2L>, public ModuleParams {
public:
  Flktr2L(const char *port, const speed_t speed = B115200);

  virtual ~Flktr2L() = default;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static Flktr2L *instantiate(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  /** @see ModuleBase::run() */
  void run() override;

  /** @see ModuleBase::print_status() */
  int print_status() override;

private:

  /**
   * Check for parameter changes and update them if needed.
   * @param parameter_update_sub uorb subscription to parameter_update
   * @param force for a parameter update
   */
  void parameters_update(bool force = false);
  //void adc();
  int open_serial_port(const char *port, const speed_t speed);

  DEFINE_PARAMETERS(
		    (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		    (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
		    )

  // Subscriptions
  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
  uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
  uORB::Publication<tool_status_s> _tool_status_pub{ORB_ID(tool_status)};

  tool_status_s _msg = {};

  bool _failsafe = false;

  char _port[20] {};
  speed_t _speed {};
  int _file_descriptor{-1};

  const uint16_t sReferenceHeader[2] = {0xFFe5, 0xFFe5};
  static const int sBufferSize = 40;

  //This must correspond 1:1 to the Package structure on the Teensy side!
  struct Package {
    uint16_t _header[2];
    uint8_t _size = sizeof(Flktr2L::Package);
    uint32_t _scale = 0;
  } _package;

  void _shiftAndAdd(uint8_t oneByte); //shift by one byte and add one byte
  uint8_t _buffer[sBufferSize];
  hrt_abstime _timestamp_last_sample {};
};
