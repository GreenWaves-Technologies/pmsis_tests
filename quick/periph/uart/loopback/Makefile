APP = test
APP_SRCS = test.c
APP_CFLAGS = -O3 -g

CONFIG_UART = 1

RUNNER_CONFIG = --config-ini=$(CURDIR)/config.ini

override runner_args += --config-opt=**/runner/peripherals=true
override runner_args += --config-opt=**/board/uart/loopback=true
override runner_args += --config-opt=**/board/uart/stdout=false

include $(RULES_DIR)/pmsis_rules.mk
