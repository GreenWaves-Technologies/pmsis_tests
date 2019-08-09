-include $(RULES_DIR)/properties.mk

RUNNER_CONFIG = $(CURDIR)/config-$(TARGET_NAME).ini

ifeq (,$(wildcard $(RUNNER_CONFIG)))
RUNNER_CONFIG = $(CURDIR)/config.ini
endif


APP = test
APP_SRCS = test.c
APP_CFLAGS += -O3 -g

include $(RULES_DIR)/pmsis_rules.mk
