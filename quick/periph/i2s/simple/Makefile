APP = test
APP_SRCS = test.c kiss_fft.c
APP_CFLAGS = -O3 -g
APP_LDFLAGS = -lm -lgcc

RUNNER_CONFIG = $(CURDIR)/config.ini

STIM_WAV_0_0 ?= $(CONFIG_BUILD_DIR)/stim_0_0.wav
STIM_WAV_0_1 ?= $(CONFIG_BUILD_DIR)/stim_0_1.wav
STIM_WAV_1_0 ?= $(CONFIG_BUILD_DIR)/stim_1_0.wav
STIM_WAV_1_1 ?= $(CONFIG_BUILD_DIR)/stim_1_1.wav

SAMPLING_FREQ_0 ?= 44100
SAMPLING_FREQ_1 ?= 22050

SIGNAL_FREQ_0_0 ?= 800
SIGNAL_FREQ_0_1 ?= 1400
SIGNAL_FREQ_1_0 ?= 2000
SIGNAL_FREQ_1_1 ?= 2600

PDM ?= 1
NB_ITF ?= 1
DUAL ?= 0
MODE ?= USE_1CH

override runner_args += --config-opt=microphone0/stim_left=$(STIM_WAV_0_0)
override runner_args += --config-opt=microphone0/stim_right=$(STIM_WAV_0_1)
override runner_args += --config-opt=microphone1/stim_left=$(STIM_WAV_1_0)
override runner_args += --config-opt=microphone1/stim_right=$(STIM_WAV_1_1)
ifeq '$(PDM)' '1'
override runner_args += --config-opt=microphone0/pdm=true
override runner_args += --config-opt=microphone1/pdm=true
else
override runner_args += --config-opt=microphone0/pdm=false
override runner_args += --config-opt=microphone1/pdm=false
endif
ifeq '$(DUAL)' '1'
override runner_args += --config-opt=microphone0/ddr=true
override runner_args += --config-opt=microphone1/ddr=true
else
override runner_args += --config-opt=microphone0/ddr=false
override runner_args += --config-opt=microphone1/ddr=false
endif

override APP_CFLAGS += -O3 -g -DSAMPLING_FREQ_0=$(SAMPLING_FREQ_0) -DSAMPLING_FREQ_1=$(SAMPLING_FREQ_1) -DSIGNAL_FREQ_0_0=$(SIGNAL_FREQ_0_0) -DSIGNAL_FREQ_0_1=$(SIGNAL_FREQ_0_1) -DSIGNAL_FREQ_1_0=$(SIGNAL_FREQ_1_0) -DSIGNAL_FREQ_1_1=$(SIGNAL_FREQ_1_1) -DPDM=$(PDM) -DNB_ITF=$(NB_ITF) -DDUAL=$(DUAL) -DMODE=$(MODE)

gen:
	sox -n -r $(SAMPLING_FREQ_0) $(STIM_WAV_0_0) synth 3 sine $(SIGNAL_FREQ_0_0)
	sox -n -r $(SAMPLING_FREQ_0) $(STIM_WAV_0_1) synth 3 sine $(SIGNAL_FREQ_0_1)
	sox -n -r $(SAMPLING_FREQ_1) $(STIM_WAV_1_0) synth 3 sine $(SIGNAL_FREQ_1_0)
	sox -n -r $(SAMPLING_FREQ_1) $(STIM_WAV_1_1) synth 3 sine $(SIGNAL_FREQ_1_1)

include $(RULES_DIR)/pmsis_rules.mk
