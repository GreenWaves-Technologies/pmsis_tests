/* 
 * Copyright (C) 2017 ETH Zurich, University of Bologna and GreenWaves Technologies
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 */

#include "pmsis.h"
#include "rtos/os_frontend_api/pmsis_task.h"
#include "stdio.h"
#include "rtos/pmsis_os.h"
#include "rtos/pmsis_driver_core_api/pmsis_driver_core_api.h"
#include "drivers/hyperbus.h"

#define BUFF_SIZE 2048

static char *buff[2];
static char *rcv_buff[2];
static uint32_t hyper_buff[2];
static int count = 0;
static struct pi_device hyper;
static struct pi_task fc_tasks[2];

static void end_of_rx(void *arg)
{
  int i = (int)arg;
  printf("End of RX for id %d\n", (int)arg);
  count++;
}

static void end_of_tx(void *arg)
{
  printf("End of TX for id %d\n", (int)arg);
  int i = (int)arg;
  pi_hyper_read_async(&hyper, hyper_buff[i], rcv_buff[i], BUFF_SIZE, pi_task_callback(&fc_tasks[i], end_of_rx, (void *)i));
}

int test_entry()
{
  printf("Entering main controller\n");

  for (int i=0; i<2; i++)
  {
    buff[i] = pmsis_l2_malloc(BUFF_SIZE);
    if (buff[i] == NULL)
      return -1;
    rcv_buff[i] = pmsis_l2_malloc(BUFF_SIZE);
    if (rcv_buff[i] == NULL)
      return -1;
  }

  struct pi_hyper_conf conf;
  pi_hyper_conf_init(&conf);

  conf.id = 0;
  conf.ram_size = 1<<20;

  pi_open_from_conf(&hyper, &conf);

  if (pi_hyper_open(&hyper))
    return -1;

  hyper_buff[0] = pi_hyperram_alloc(&hyper, BUFF_SIZE);
  if (hyper_buff[0] == 0) return -1;

  hyper_buff[1] = pi_hyperram_alloc(&hyper, BUFF_SIZE);
  if (hyper_buff[1] == 0) return -1;

  for (int i=0; i<BUFF_SIZE; i++)
    {
      buff[0][i] = i & 0x7f;
      buff[1][i] = i | 0x80;
    }

  //  printf("%s %d\n", __FILE__,__LINE__);
  pi_hyper_write_async(&hyper, hyper_buff[0], buff[0], BUFF_SIZE, pi_task_callback(&fc_tasks[0], end_of_tx, (void *)0));

  pi_hyper_write_async(&hyper, hyper_buff[1], buff[1], BUFF_SIZE, pi_task_callback(&fc_tasks[1], end_of_tx, (void *)1));


  while(count != 2) {
    pi_yield();
  }

  for (int j=0; j<2; j++)
  {
    for (int i=0; i<BUFF_SIZE; i++)
    {
      unsigned char expected;
      if (j == 0)
        expected = i & 0x7f;
      else
        expected = i | 0x80;
      if (expected != rcv_buff[j][i])
      {
        printf("Error, buffer: %d, index: %d, expected: 0x%x, read: 0x%x\n", j, i, expected, rcv_buff[j][i]);
        return -1;
      }
    }
  }

  pi_hyperram_free(&hyper, hyper_buff[0], BUFF_SIZE);
  pi_hyperram_free(&hyper, hyper_buff[1], BUFF_SIZE);
  pi_hyper_close(&hyper);

  return 0;
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