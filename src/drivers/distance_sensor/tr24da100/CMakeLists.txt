############################################################################
#
#   Copyright (c) 2023 Fluktor GmbH. All rights reserved.
#
############################################################################
px4_add_module(
	MODULE drivers__distance_sensor__tr24da100
	MAIN tr24da100
	SRCS
		tr24da100.cpp
		tr24da100.hpp
		tr24da100_main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_rangefinder
		px4_work_queue
	)
