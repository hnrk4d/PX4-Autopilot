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
#include <uORB/topics/tool_status.h>
#include <uORB/topics/vehicle_command.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

using namespace time_literals;

extern "C" __EXPORT int tooldrv_main(int argc, char *argv[]);

class ToolDrv : public ModuleBase<ToolDrv>, public ModuleParams {
public:
  ToolDrv(const char *port, const speed_t speed = B115200);

  virtual ~ToolDrv() = default;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static ToolDrv *instantiate(int argc, char *argv[]);

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
  uORB::Publication<tool_status_s> _tool_status_pub{ORB_ID(tool_status)};
  uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
  tool_status_s _msg_tool_status = {};
  PX4Rangefinder _px4_rangefinder_forward;
  PX4Rangefinder _px4_rangefinder_downward;

  char _port[20] {};
  speed_t _speed {};
  int _file_descriptor{-1};

  const uint16_t sReferenceHeader[2] = {0xFFe5, 0xFFe3};
  static const int sBufferSize = 40;

  float _cruise_speed = 0;
  float _cruise_speed_default = 0;
  bool _failsafe = false;
  uint8_t  _nav_state = -1;
  uint8_t _arming_state = 0;
  float _mission_aux[6] {}; //last aux setting in a mission state
  float _cache_aux[6] {};
  bool _paused = false;

  //This must correspond 1:1 to the PckgTeensy2PX4 structure on the Teensy side!
  struct PckgTeensy2PX4 {
    enum {DOWNWARD=0x01, FORWARD=0x02, SCALE=0x04, PWM=0x08};
    uint16_t _header[2];
    uint16_t _crc = 0;
    uint8_t _size = sizeof(ToolDrv::PckgTeensy2PX4);
    uint32_t _mod = 0;
    float _downward_dist = 0.0f;
    float _forward_dist = 0.0f;
    uint32_t _scale = 0;
    uint8_t _pwm[6];
  } _teensy2px4;

  struct PckgPX42Teensy {
    enum {TARGET_SPEED=0x0001, ACTUAL_SPEED=0x0002, AUX1=0x0004, AUX2=0x0008, AUX3=0x0010, AUX4=0x0020, AUX5=0x0040, AUX6=0x0080, TEST=0x0100, STATUS=0x0200};
    const uint16_t _header[2] = {0xFFe5, 0xFFe3};
    uint16_t _crc;
    const uint8_t _size = sizeof(ToolDrv::PckgPX42Teensy);
    uint32_t _mod;
    float _target_speed;
    float _actual_speed;
    bool _failsafe;
    uint8_t  _nav_state;
    uint8_t _arming_state;
    bool _test;
    float _aux[6];
    void reset() {
      _target_speed = 0;
      _actual_speed = 0;
      _mod = 0;
      _failsafe = false;
      _nav_state = 0;
      _arming_state = 1;
      _test = false;
      _crc = 0;
      for(auto &x : _aux) x = -1;
    }
  } _px42teensy;

  uint16_t crc16(char *data_p, uint16_t length);
  void _shiftAndAdd(uint8_t oneByte); //shift by one byte and add one byte
  uint8_t _buffer[sBufferSize];
  hrt_abstime _timestamp_last_sample {};
  hrt_abstime _timestamp_last_write {};
  hrt_abstime _timestamp_last_speed {};
  hrt_abstime _timestamp_last_speed_log {};
};
