-include $(RULES_DIR)/properties.mk

CONFIG_SPIM=1

RUNNER_CONFIG = $(CURDIR)/config-$(TARGET_NAME).ini

ifeq (,$(wildcard $(RUNNER_CONFIG)))
RUNNER_CONFIG = $(CURDIR)/config.ini
endif

ifeq '$(TARGET_NAME)' 'gap9_v2'
APP_CFLAGS += -DSPIM_ITF=0
APP_CFLAGS += -DSPIM_CS=0
else
ifeq '$(TARGET_NAME)' 'wolfe'
APP_CFLAGS += -DSPIM_ITF=0
else
ifeq '$(TARGET_NAME)' 'pulp'
APP_CFLAGS += -DSPIM_ITF=0
else
ifeq '$(TARGET_NAME)' 'pulpissimo_v1'
APP_CFLAGS += -DSPIM_ITF=0
else
ifeq '$(TARGET_NAME)' 'pulpissimo'
APP_CFLAGS += -DSPIM_ITF=0
else
ifeq '$(TARGET_NAME)' 'vivosoc3'
APP_CFLAGS += -DSPIM_ITF=1
else
APP_CFLAGS += -DSPIM_ITF=1
endif
endif
endif
endif
endif
endif

override CONFIG_OPT += runner/peripherals=true


APP = test
APP_SRCS = test.c
APP_CFLAGS += -O3 -g

include $(RULES_DIR)/pmsis_rules.mk
