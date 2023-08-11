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
#include <uORB/topics/adc_report.h>

#include <parameters/param.h>

int Flktr2L::print_status() {
  PX4_INFO("Running, failsafe status : %d", _failsafe);

  return 0;
}

int Flktr2L::custom_command(int argc, char *argv[]) {
    if (!is_running()) {
      print_usage("not running");
      return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "adc")) {

    get_instance()->adc();
    return 0;
    }

  return print_usage("unknown command");
}


int Flktr2L::task_spawn(int argc, char *argv[]) {
  _task_id = px4_task_spawn_cmd("module",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				1024,
				(px4_main_t)&run_trampoline,
				(char *const *)argv);

  if (_task_id < 0) {
    _task_id = -1;
    return -errno;
  }

  return 0;
}

Flktr2L *Flktr2L::instantiate(int argc, char *argv[]) {
  bool error_flag = false;

  int myoptind = 1;
  int ch;
  const char *myoptarg = nullptr;

  // parse CLI arguments
  while ((ch = px4_getopt(argc, argv, "", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case '?':
      error_flag = true;
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

  Flktr2L *instance = new Flktr2L();

  if (instance == nullptr) {
    PX4_ERR("alloc failed");
  }

  return instance;
}

Flktr2L::Flktr2L()
  : ModuleParams(nullptr) {
}

void Flktr2L::run() {
  struct vehicle_status_s vehicle_status;
  struct adc_report_s adc_report;

  int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  int adc_report_sub = orb_subscribe(ORB_ID(adc_report));

  px4_pollfd_struct_t fds[2];
  fds[0].fd = vehicle_status_sub;
  fds[0].events = POLLIN;
  fds[1].fd = adc_report_sub;
  fds[1].events = POLLIN;

  // initialize parameters
  parameters_update(true);

  hrt_abstime _timestamp_last_adc_report = hrt_absolute_time();
  
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

      parameters_update();
    }
    px4_usleep(100000);
  }

  orb_unsubscribe(vehicle_status_sub);
  orb_unsubscribe(adc_report_sub);
}

void Flktr2L::adc() {
  PX4_INFO("ADC: %ld %ld",
	_msg.data_1,
	_msg.data_2
  );
}

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
The module flktr2l supports start/stop/status/adc functionality and runs in the background.

### Implementation
The module flktr2l is listening to the vehicle and tool status. If a failsafe status is triggered, the actuators will be turned off. Depending on the tool it collects tool depending information.

### Examples
CLI usage example:
$ flktr2l start

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("module", "template");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int flktr2l_main(int argc, char *argv[]) {
  return Flktr2L::main(argc, argv);
}
