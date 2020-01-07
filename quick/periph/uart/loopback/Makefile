PULP_APP = test
PULP_APP_FC_SRCS = test.c
PULP_CFLAGS = -O3 -g

CONFIG_UART = 1

PULP_PROPERTIES += pulp_chip
include $(PULP_SDK_HOME)/install/rules/pulp_properties.mk

ifeq '$(pulp_chip)' 'vivosoc3'
USER_CONFIG = $(CURDIR)/config-periph.ini
endif

override CONFIG_OPT += runner/peripherals=true
override runner_args += --config-opt=**/board/uart/loopback=true
override runner_args += --config-opt=**/board/uart/stdout=false

include $(PULP_SDK_HOME)/install/rules/pulp_rt.mk
