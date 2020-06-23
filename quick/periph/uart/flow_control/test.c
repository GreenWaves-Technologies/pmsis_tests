/* 
 * Copyright (C) 2017 GreenWaves Technologies
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 */

#include "pmsis.h"
#include "testbench.h"
#include <string.h>

struct pi_uart_conf conf;
struct pi_device uart;
static pi_device_t bench;

#define MAX_ITER 16
#define BUFFER_SIZE  4
#define START_VALUE 0x73
#define INCR_VALUE  0x7

static PI_L2 uint8_t tx_buffer[BUFFER_SIZE];
static PI_L2 uint8_t rx_buffer[2][BUFFER_SIZE];
static pi_task_t tasks[MAX_ITER];
static int rx_buffer_index;
static pi_task_t rx_task;
static uint8_t rx_current_val;
static int rx_errors;
static int rx_total_size;


static void end_of_rx(void *arg)
{
    pi_uart_read_async(&uart, rx_buffer[rx_buffer_index], BUFFER_SIZE, pi_task_callback(&rx_task, end_of_rx, NULL));
    rx_buffer_index ^=1;
    rx_total_size += BUFFER_SIZE;

    for (int i=0; i<BUFFER_SIZE; i++)
    {
        if (rx_buffer[rx_buffer_index][i] != rx_current_val)
        {
            rx_errors++;
        }

        rx_current_val += INCR_VALUE;
    }

}


static int test_uart()
{
    int start = START_VALUE;
    int incr = INCR_VALUE;
    int baudrate = 10000000;
    int bandwidth = baudrate / 8 / 2;
    int nb_iter = 1;
    int size = nb_iter * BUFFER_SIZE;

    pi_uart_conf_init(&conf);

    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.uart_id = 1;
    conf.use_ctrl_flow = 1;
    conf.baudrate_bps = baudrate;

    pi_pad_set_function(67, 2);
    pi_pad_set_function(68, 2);

    //pi_freq_set(PI_FREQ_DOMAIN_FC, 100000);

    pi_testbench_uart_traffic_gen(&bench, &uart, &conf);

    char command[256];

    uint32_t value = start;
    for (int i=0; i<BUFFER_SIZE; i++)
    {
        tx_buffer[i] = value;
        value += incr;
    }

    memset(rx_buffer, 0, BUFFER_SIZE);

    // Command to activate RX traffic
    pi_testbench_uart_traffic_gen_rx(&uart, start, incr, size, bandwidth, nb_iter);

    // Command to activate TX traffic
    pi_testbench_uart_traffic_gen_tx(&uart, start, incr, size);

    // Start
    pi_testbench_uart_traffic_gen_start(&uart);


    rx_total_size = 0;
    rx_errors = 0;
    rx_current_val = START_VALUE;
    rx_buffer_index = 1;
    pi_uart_read_async(&uart, rx_buffer[0], BUFFER_SIZE, pi_task_callback(&rx_task, end_of_rx, NULL));



    int index = 0;
    while (size > 0)
    {
        int iter_size = BUFFER_SIZE;
        if (iter_size > size)
            iter_size = size;

        pi_uart_write_async(&uart, tx_buffer, BUFFER_SIZE, pi_task_block(&tasks[index++]));

        size -= iter_size;
    }


    for (int i=0; i<index; i++)
    {
        pi_task_wait_on(&tasks[i]);
    }

    while(rx_total_size < BUFFER_SIZE * nb_iter)
    {
        pi_yield();
    }

    pi_uart_ioctl(&uart, PI_UART_IOCTL_ABORT_RX, NULL);

    if (pi_testbench_uart_traffic_gen_status(&uart))
        rx_errors++;

    return rx_errors;
}


static int test_entry()
{
    printf("Entering test\n");

    int errors = 0;
    int index = 0;

    struct pi_testbench_conf conf;

    pi_testbench_conf_init(&conf);

    pi_open_from_conf(&bench, &conf);

    if (pi_testbench_open(&bench))
        return -1;

    errors += test_uart();

    return errors;
}

static void test_kickoff(void *arg)
{
    int ret = test_entry();
    pmsis_exit(ret);
}

int main()
{
    return pmsis_kickoff((void *)test_kickoff);
}