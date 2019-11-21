/*
 * Copyright (C) 2018 ETH Zurich and University of Bologna and
 * GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pmsis.h"
#include "kiss_fft.h"



#ifndef NB_ITF
#define NB_ITF 1
#endif

#ifndef NB_CHANNEL
#define NB_CHANNEL 1
#endif

#define ERROR_RATE 20
#define ELEM_SIZE 2
#define NB_ELEM 256
#define BUFF_SIZE ((NB_ELEM+16)*ELEM_SIZE)
#define BLOCK_SIZE (NB_ELEM*ELEM_SIZE)

static uint8_t buff[NB_ITF][2][BUFF_SIZE*NB_CHANNEL];



static kiss_fft_cpx buff_complex[NB_ELEM];
static kiss_fft_cpx buff_complex_out[NB_ELEM];
static kiss_fft_cfg cfg;

static int get_sampling_freq(int itf)
{
  if (itf == 0)
    return SAMPLING_FREQ_0;
  else
    return SAMPLING_FREQ_1;
}

static int get_signal_freq(int itf, int channel)
{
  if (itf == 0)
  {
#if defined(MODE) && MODE == USE_2CH
    if (channel == 0)
      return SIGNAL_FREQ_0_0;
    else if (channel == 2)
      return SIGNAL_FREQ_0_1;
    else if (channel == 1)
      return SIGNAL_FREQ_1_0;
    else
      return SIGNAL_FREQ_1_1;
#else
    if (channel == 0)
      return SIGNAL_FREQ_0_0;
    else if (channel == 1)
      return SIGNAL_FREQ_0_1;
#endif
  }
  else
    if (channel == 0)
      return SIGNAL_FREQ_1_0;
    else
      return SIGNAL_FREQ_1_1;
}

static int check_buffer(uint8_t *buff, int sampling_freq, int signal_freq)
{
  for (int i=0; i<NB_ELEM; i++)
  {
    //printf("%x\n", (((int16_t *)buff)[i+16]));

    buff_complex[i].r = (float)(((int16_t *)buff)[i+16]);
    buff_complex[i].i = 0.0;
  }

  // Get signal frequencies
  kiss_fft(cfg, buff_complex, buff_complex_out);

  float max = 0;
  int index = -1;

  // Find the most important one
  for (int i=0; i<NB_ELEM; i++)
  {
    float val = buff_complex_out[i].r*buff_complex_out[i].r*1.0/sampling_freq + buff_complex_out[i].i*buff_complex_out[i].i*1.0/sampling_freq;
    if (index == -1 || val > max)
    {
      index = i;
      max = val;
    }
  }

  float freq = (int)((float)index / NB_ELEM * sampling_freq);
  float error = (freq - signal_freq) / signal_freq * 100;
  if (error < 0)
    error = -error;

  printf("Got error rate %d %% (expected: %d, got %d)\n", (int)error, (int)signal_freq, (int)freq);

  return error > ERROR_RATE;
}

static int test_entry()
{
  struct pi_device i2s[NB_ITF];
  int errors = 0;

  printf("Entering main controller\n");

  cfg = kiss_fft_alloc(NB_ELEM, 0, NULL, NULL);

  for (int i=0; i<NB_ITF; i++)
  {
    struct pi_i2s_conf i2s_conf;
    pi_i2s_conf_init(&i2s_conf);

    i2s_conf.pingpong_buffers[0] = buff[i][0];
    i2s_conf.pingpong_buffers[1] = buff[i][1];
    i2s_conf.block_size = BLOCK_SIZE;
    i2s_conf.frame_clk_freq = get_sampling_freq(i);
    i2s_conf.itf = i;
    i2s_conf.format = PI_I2S_FMT_DATA_FORMAT_PDM;
    i2s_conf.pdm_decimation_log2 = 8;
    i2s_conf.word_size = 16;

    pi_open_from_conf(&i2s[i], &i2s_conf);

    if (pi_i2s_open(&i2s[i]))
      return -1;
  }

  for (int i=0; i<NB_ITF; i++)
  {
    if (pi_i2s_ioctl(&i2s[i], PI_I2S_IOCTL_START, NULL))
      return -1;
  }

  void *chunk[NB_ITF];
  for (int i=0; i<NB_ITF; i++)
  {
    int size;
    pi_i2s_read(&i2s[i], &chunk[i], &size);
    pi_i2s_ioctl(&i2s[i], PI_I2S_IOCTL_STOP, NULL);
    pi_i2s_close(&i2s[i]);
  }

#if 0
  for (int j=0; j<BUFF_SIZE/2; j++)
  {
    short *buff0 = buff[0][1];
    printf("%d\n", buff0[j]);
  }
#endif

  for (int i=0; i<NB_ITF; i++)
  {
    for (int k=0; k<NB_CHANNEL; k++)
    {
      errors += check_buffer(buff[i][1], get_sampling_freq(i), get_signal_freq(i, k));
    }
  }

  if (errors)
    printf("TEST FAILURE\n");
  else
    printf("TEST SUCCESS\n");

  return errors;
}

void test_kickoff(void *arg)
{
  int ret = test_entry();
  pmsis_exit(ret);
}

int main()
{
  return pmsis_kickoff((void *)test_kickoff);
}