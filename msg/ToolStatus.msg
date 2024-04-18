uint8 TOOL_NONE = 0
uint8 TOOL_SPREADER_LEHNER = 1
uint8 TOOL_SPRAYER_AGROTOP = 2

uint64 timestamp			# time since system start (milliseconds)

uint8 id				# Tool ID
int32 weight				# contains the tank weight
int32 data				# generic data

uint16 aux                              # bit mask for AUX channel that was modified
uint8 pwm_0                             # PWM status in the order provided by the microcontroller tool driver
uint8 pwm_1				# must not correspond to the PWM order of the microcontroller nor AUX channels
uint8 pwm_2
uint8 pwm_3
uint8 pwm_4
uint8 pwm_5
