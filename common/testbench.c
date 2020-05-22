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

#define PI_TESTBENCH_CMD_GPIO_LOOPBACK 1

typedef struct {
    pi_device_t ctrl_dev;
} pi_testbench_t;


typedef struct {
    uint8_t cmd;
} pi_testbench_req_header_t;


typedef struct {
    uint8_t input;
    uint8_t output;
    uint8_t enabled;
} pi_testbench_req_t;



int pi_testbench_open(struct pi_device *device)
{
    struct pi_testbench_conf *conf = (struct pi_testbench_conf *)device->config;

    pi_testbench_t *bench = pi_l2_malloc(sizeof(pi_testbench_t));
    if (bench == NULL)
        return -1;

    device->data = (void *)bench;

    struct pi_uart_conf uart_conf;

    pi_uart_conf_init(&uart_conf);

    pi_open_from_conf(&bench->ctrl_dev, &uart_conf);

    if (pi_uart_open(&bench->ctrl_dev))
    {
        pi_l2_free(bench, sizeof(pi_testbench_t));
        return -1;
    }

    return 0;
}


void pi_testbench_conf_init(struct pi_testbench_conf *conf)
{
    conf->ctrl_type = PI_TESTBENCH_CTRL_TYPE_UART;
    conf->uart_baudrate = 115200;
    conf->uart_id = 0;
}


void pi_testbench_gpio_loopback(pi_device_t *device, int output, int input, int enabled)
{
    pi_testbench_t *bench = (pi_testbench_t *)device->data;

    pi_testbench_req_header_t header = { .cmd = PI_TESTBENCH_CMD_GPIO_LOOPBACK };
    pi_uart_write(&bench->ctrl_dev, &header, sizeof(header));

    pi_testbench_req_t req = { .output=output, .input=input, .enabled=enabled };
    pi_uart_write(&bench->ctrl_dev, &req, sizeof(req));

    pi_uart_ioctl(&bench->ctrl_dev, PI_UART_IOCTL_FLUSH, NULL);
}
