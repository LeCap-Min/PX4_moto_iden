############################################################################
#
# FashionStar UART servo bus driver
#
############################################################################

px4_add_module(
	MODULE drivers__fs_uart_servo
	MAIN fs_uart_servo
	COMPILE_FLAGS
	SRCS
		FsUartServo.cpp
		FsUartServo.hpp
		fs_protocol.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)
