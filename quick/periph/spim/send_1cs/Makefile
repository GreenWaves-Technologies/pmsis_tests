-include $(RULES_DIR)/properties.mk

CONFIG_SPIM=1

RUNNER_CONFIG := $(CURDIR)/config-$(TARGET_NAME).ini

ifeq (,$(wildcard $(RUNNER_CONFIG)))
RUNNER_CONFIG = $(CURDIR)/config.ini
endif

APP_CFLAGS = $(TEST_CFLAGS)

ifeq '$(TARGET_NAME)' 'gap9_v2'
APP_CFLAGS += -DSPIM_ITF=2
APP_CFLAGS += -DSPIM_CS=0
else
ifeq '$(TARGET_NAME)' 'vega'
APP_CFLAGS += -DSPIM_ITF=1
APP_CFLAGS += -DSPIM_CS=0
else
ifeq '$(TARGET_NAME)' 'vivosoc3'
APP_CFLAGS += -DSPIM_ITF=0
APP_CFLAGS += -DSPIM_CS=1
else
APP_CFLAGS += -DSPIM_ITF=0
APP_CFLAGS += -DSPIM_CS=1
endif
endif
endif

APP = test
APP_SRCS += test.c
APP_CFLAGS += -Os -g

include $(RULES_DIR)/pmsis_rules.mk
