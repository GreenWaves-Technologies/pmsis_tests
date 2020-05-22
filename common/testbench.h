/* 
 * Copyright (C) 2017 GreenWaves Technologies
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 */

#include "pmsis.h"

enum pi_testbench_ctrl_type
{
    PI_TESTBENCH_CTRL_TYPE_UART
};


struct pi_testbench_conf
{
    enum pi_testbench_ctrl_type ctrl_type;
    int uart_baudrate;
    int uart_id;
};

int pi_testbench_open(struct pi_device *device);

void pi_testbench_conf_init(struct pi_testbench_conf *conf);

void pi_testbench_gpio_loopback(pi_device_t *device, int output, int input, int enabled);
