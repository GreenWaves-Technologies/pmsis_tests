APP = test
APP_SRCS = test.c kiss_fft.c
APP_CFLAGS = -O3 -g
APP_LDFLAGS = -lm -lgcc

CONFIG_I2S = 1

override config_args += --ini-config=$(CURDIR)/config.ini

ifdef GAP_SDK_HOME
CONFIG_BUILD_DIR = $(BUILDDIR)
endif

#RUNNER_CONFIG = $(CURDIR)/config.ini

STIM_WAV_0_0 ?= $(CONFIG_BUILD_DIR)/stim_0_0.wav
STIM_WAV_0_1 ?= $(CONFIG_BUILD_DIR)/stim_0_1.wav
STIM_WAV_0_2 ?= $(CONFIG_BUILD_DIR)/stim_0_2.wav
STIM_WAV_0_3 ?= $(CONFIG_BUILD_DIR)/stim_0_3.wav


SAMPLING_FREQ_0 ?= 44100
SAMPLING_FREQ_1 ?= 44100

SIGNAL_FREQ_0_0 ?= 800
SIGNAL_FREQ_0_1 ?= 1400
SIGNAL_FREQ_0_2 ?= 2000
SIGNAL_FREQ_0_3 ?= 2600

PDM ?= 1
NB_ITF ?= 1
NB_CHANNELS ?= 1
WORD_SIZE ?= 16
TDM ?= 0

override config_args += --config-item=microphone0/width=$(WORD_SIZE)

override config_args += --config-item=microphone0/stim_0=$(STIM_WAV_0_0)
override config_args += --config-item=microphone0/stim_1=$(STIM_WAV_0_1)
override config_args += --config-item=microphone0/stim_2=$(STIM_WAV_0_2)
override config_args += --config-item=microphone0/stim_3=$(STIM_WAV_0_3)


override APP_CFLAGS += -O3 -g -DSAMPLING_FREQ_0=$(SAMPLING_FREQ_0) -DSAMPLING_FREQ_1=$(SAMPLING_FREQ_1) \
	-DSIGNAL_FREQ_0_0=$(SIGNAL_FREQ_0_0) -DSIGNAL_FREQ_0_1=$(SIGNAL_FREQ_0_1) -DSIGNAL_FREQ_0_2=$(SIGNAL_FREQ_0_2) \
	-DSIGNAL_FREQ_0_3=$(SIGNAL_FREQ_0_3) -DPDM=$(PDM) -DNB_ITF=$(NB_ITF) -DNB_CHANNELS=$(NB_CHANNELS) \
	-DWORD_SIZE=$(WORD_SIZE) -DTDM=$(TDM)

gen:
	sox -n -r $(SAMPLING_FREQ_0) --bits $(WORD_SIZE) $(STIM_WAV_0_0) synth 3 sine $(SIGNAL_FREQ_0_0) vol 0.995
	sox -n -r $(SAMPLING_FREQ_0) --bits $(WORD_SIZE) $(STIM_WAV_0_1) synth 3 sine $(SIGNAL_FREQ_0_1) vol 0.995
	sox -n -r $(SAMPLING_FREQ_0) --bits $(WORD_SIZE) $(STIM_WAV_0_2) synth 3 sine $(SIGNAL_FREQ_0_2) vol 0.995
	sox -n -r $(SAMPLING_FREQ_0) --bits $(WORD_SIZE) $(STIM_WAV_0_3) synth 3 sine $(SIGNAL_FREQ_0_3) vol 0.995

include $(RULES_DIR)/pmsis_rules.mk
