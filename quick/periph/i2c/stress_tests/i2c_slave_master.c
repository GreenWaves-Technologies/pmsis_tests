/**
 * I2C Stress Test
 *
 * Small eeprom like protocol but with L2
 * Leader send 32 bit address + 32 bit size
 * Slave send back content at that address
 *
 * This is done through the i2c_memory_rx_callback which is registered to be
 * called at each rx end of frame
 *
 * On the tx side, handler is just a dummy, as nothing is really needed there
 * (tx transfer will clear itself when transfer is done)
 *
 **/

#include <stdio.h>
#include <stdbool.h>

#include "pmsis.h"
#include "pmsis/drivers/i2c.h"
#include "pmsis/drivers/i2c_slave.h"

#define I2C_SLAVE_L2_TEST_ADDRESS (0x1c019000)
#define I2C_SLAVE_ADDR0 (0x14) // just some random address
#define I2C_SLAVE_ADDR0_10BITS (0x114)
#define TEST_VALUE_BASE (0x01)

#define INTERFACE_SLAVE   (1)
#define INTERFACE_MASTER0  (0)
#define INTERFACE_MASTER1  (2)
#define MASTER_ENABLED (1)

#define PRINTF(...)
//#define PRINTF(...) printf(__VA_ARGS__)

// 64 * 4 bytes
#define BUFF_SIZE (16)
#define I2C_SLAVE_L2_TEST_SIZE (BUFF_SIZE*4)


// ===================================================
// STATIC VARIABLES
// ===================================================

/* slave device */
static pi_device_t* slave_dev = NULL;

/* buffer to hold rx on slave side */
/* BUFF SIZE + addr + size + an extra byte for matcher */
static uint8_t rx_buffer[((BUFF_SIZE+2)*4)+1] = {0};

/* current address counter */
/* Address goes from 0 to BUFF_SIZE*4 */
static int32_t eeprom_current_address = 0;

/* buffer containing EEPROM memory */
static uint8_t eeprom_memory[BUFF_SIZE * 4];


/* master device */
static pi_device_t* master_dev0 = NULL;
static pi_device_t* master_dev1 = NULL;
/* master async read buffer */
static uint8_t async_read_buff[2] = { 0 , 0 };


// ===================================================
// FUNCTIONS
// ===================================================

void eeprom_data_init(void)
{
    // set the whole addr field to some ducky value
    // allows to check if transfer makes sense
    rx_buffer[1] = 0x7F;
    rx_buffer[2] = 0x7F;
    rx_buffer[3] = 0x7F;
    rx_buffer[4] = 0x7F;

    for (int i = 0; i < BUFF_SIZE * 4; i++)
    {
        eeprom_memory[i] = 0xef;
    }
}

static void dump_rx(void)
{
    for (int i = 0; i < ((BUFF_SIZE+2)*4)+1; i++)
    {
        PRINTF("%x ", rx_buffer[i]);
    }
    PRINTF("\n");
}

static void dump_eeprom_memory(void)
{
    for (int i = 0; i < BUFF_SIZE*4; i++)
    {
        PRINTF("%x ", eeprom_memory[i]);
    }
    PRINTF("\n");
}

/* flag for async read/write */
static volatile uint8_t done = 0;
static volatile uint8_t async_error = 0;

/* Callback for end of RX transfer. */
static void __end_of_rx(void *arg)
{
    pi_task_t* task = (pi_task_t *) arg;
    int status = pi_i2c_get_request_status(task);
    async_error = 0;
    if (status != PI_OK)
    {
        async_error = status;
    }
    done = 1;
}

/* Callback for end of TX transfer. */
static void __end_of_tx(void *arg)
{
    pi_task_t* task = (pi_task_t *) arg;
    pi_task_t* cb = (pi_task_t *) task->arg[2];
    int status = pi_i2c_get_request_status(task);
    if (status != PI_OK)
    {
        async_error = status;
        done =1;
    }
    else
    {
        pi_i2c_read_async(master_dev0, async_read_buff, 2, PI_I2C_XFER_START | PI_I2C_XFER_STOP, cb);
    }
}

/**
 * \brief callback called when data is received
 *
 * ie. a write
 */
void i2c_memory_emu_rx_callback(pi_i2c_slave_args_t *arg)
{
    PRINTF("%s:%d\n",__func__,__LINE__);
    uint32_t l2_addr = *(uint32_t*)arg->l2_buffer;
    uint32_t size = *(uint32_t*)(arg->l2_buffer + 4);

    //DEBUG
    dump_rx();

    //TODO print received buffer

    /* first 1 byte of the request contains the slave address */
    /* next 2 bytes should contain the address (LSB MSB)*/
    /* next bytes are data to write */
    /* current address is increased for each byte of data */

    /* TODO how to retrieve the rx_buffer current index ? */

    PRINTF("nb_bytes = %d\n",arg->nb_bytes);

    uint16_t slave_addr = arg->slave_addr;
    PRINTF("--> RX slave addr: %d\n", slave_addr);

    bool in_range = true; // indicate whether the write is in range

    if (arg->nb_bytes >= 2) {
        PRINTF("-> RX contains memory addr\n");
        uint16_t memory_addr = rx_buffer[1] | rx_buffer[2] << 8;
        PRINTF("--> RX memory addr: %d\n", memory_addr);
        /* check that addr is in range */
        if (memory_addr > BUFF_SIZE * 4)
        {
            memory_addr = 0;
            in_range = false;
        }
        if (in_range)
        {
            eeprom_current_address = memory_addr;
        }
    }

    if (arg->nb_bytes >= 3) {
        PRINTF("-> RX contains data\n");
        if (in_range)
        {
            /* write data in memory to eeprom_current_address */
            PRINTF("Writing data to EEPROM memory\n");
            //TODO memcpy
            uint32_t eeprom_index = 0;
            for (uint32_t i = 2; i < arg->nb_bytes; i++)
            {
                eeprom_index = i - 2 + eeprom_current_address;
                if (eeprom_index > BUFF_SIZE * 4)
                {
                    break;
                }
                eeprom_memory[eeprom_index] = ((uint8_t*)arg->l2_buffer)[i];
                PRINTF("%x ", eeprom_memory[eeprom_index]);
            }
            PRINTF("\n");
        }
    }

    pi_i2c_slave_stop_rx(arg->handle);
    pi_i2c_slave_stop_tx(arg->handle);
    pi_i2c_slave_unlock(arg->handle, 0);
    pi_i2c_slave_set_tx(arg->handle, &eeprom_memory[eeprom_current_address],
            BUFF_SIZE * 4 - eeprom_current_address + 1);
    pi_i2c_slave_set_rx(arg->handle, rx_buffer, ((BUFF_SIZE+2)*4)+1);

    arg->ret = 0;
}


/**
 * \brief callback called when data is sent
 *
 * ie. we respond to a read
 */
void i2c_memory_emu_tx_callback(pi_i2c_slave_args_t *arg)
{
    PRINTF("%s:%d\n",__func__,__LINE__);
    PRINTF("nb_bytes = %x\n",arg->nb_bytes);
    PRINTF("%s:%d\n",__func__,__LINE__);

    // nothing to do for this emulation except unlocking and reloading buf
    pi_i2c_slave_stop_tx(arg->handle);
    pi_i2c_slave_stop_rx(arg->handle);
    pi_i2c_slave_unlock(arg->handle, 0);
    pi_i2c_slave_set_tx(arg->handle, &eeprom_memory[eeprom_current_address], BUFF_SIZE*4);
    pi_i2c_slave_set_rx(arg->handle, rx_buffer, ((BUFF_SIZE+2)*4)+1);
    arg->ret = 0;

    dump_eeprom_memory();
}

/**
 * \brief setup slave to act as an EEPROM
 *
 * Setup 7 and 10 bits addresses since I2C supports 2 addresses slots
 *
 * \param slave_device pointer to the slave device
 * \param i2c_interface the interface to be used
 */
int slave_setup(pi_device_t* slave_device, uint8_t i2c_interface)
{
    struct pi_i2c_slave_conf *conf = pi_l2_malloc(sizeof(struct pi_i2c_slave_conf));
    //TODO free the configuration ?

    /* Initialize data */
    eeprom_data_init();

    /* Initialize the configuration */
    pi_i2c_slave_conf_init(conf);
    conf->itf = i2c_interface;

    /* set addresses (use both slots) */
    pi_i2c_slave_conf_set_addr0(conf, I2C_SLAVE_ADDR0, 0x1F, 0, 1, 0);
    pi_i2c_slave_conf_set_addr1(conf, I2C_SLAVE_ADDR0_10BITS, 0x1F, 1, 1, 0);

    /* setup RX and TX callbacks */
    conf->rx_callback = i2c_memory_emu_rx_callback;
    conf->tx_callback = i2c_memory_emu_tx_callback;

    pi_open_from_conf(slave_device, conf);
    int ret = pi_i2c_slave_open(slave_device);
    if(!ret)
    {
        pi_i2c_slave_set_rx(slave_device->data, rx_buffer, ((BUFF_SIZE+2)*4)+1);
        pi_i2c_slave_set_tx(slave_device->data, eeprom_memory, BUFF_SIZE*4);
        PRINTF("Slave correctly configured on I2C interface %d\n", i2c_interface);
    }
    else
    {
        PRINTF("slave open failed\n");
    }

    return ret;
}

/**
 * \brief launch the slave
 *
 * \returns -1 if initialization failed
 *          else 0
 */
int slave_launch(void)
{
    slave_dev = pi_l2_malloc(sizeof(pi_device_t));

    if (slave_setup(slave_dev, INTERFACE_SLAVE))
    {
        PRINTF("Slave 1 configuration failed!\n");
        pi_l2_free(slave_dev, sizeof(pi_device_t));
        return -1;
    }

    return 0;
}

/**
 * \brief close the slave
 *
 * \returns -1 if operation failed
 *          else 0
 */
int slave_close(void)
{
    PRINTF("Closing slave.\n");
    pi_i2c_slave_close(slave_dev);
    pi_l2_free(slave_dev, sizeof(pi_device_t));
    return 0;
}

/**
 * \brief setup the master device
 *
 * \returns -1 if initialization failed
 *          else 0
 */
int master_setup(pi_device_t* master_device, uint8_t i2c_interface, uint16_t slave_addr, int is_10bits)
{
    int status = 0;
    struct pi_i2c_conf* conf = pi_l2_malloc(sizeof(struct pi_i2c_conf));
    //TODO free the configuration ?
    pi_i2c_conf_init(conf);

    conf->itf = i2c_interface;
    /* set targe slave address */
    pi_i2c_conf_set_slave_addr(conf, slave_addr, is_10bits);
    pi_i2c_conf_set_wait_cycles(conf, 0);

    pi_open_from_conf(master_device, conf);

    if(pi_i2c_open(master_device))
    {
        PRINTF("Error while opening the I2C interface\n");
        status = -1;
    }

    return status;
}

/**
 * \brief launch the master
 *
 * \returns -1 if initialization failed
 *          else 0
 */
int masters_launch(void)
{
    /* initialize device */
    master_dev0 = pi_l2_malloc(sizeof(pi_device_t));
    master_dev1 = pi_l2_malloc(sizeof(pi_device_t));

    if (master_setup(master_dev0, INTERFACE_MASTER0, I2C_SLAVE_ADDR0, 0))
    {
        PRINTF("Master 0 configuration failed!\n");
        return -1;
    }

    if (master_setup(master_dev1, INTERFACE_MASTER1, I2C_SLAVE_ADDR0_10BITS, 1))
    {
        PRINTF("Master 1 configuration failed!\n");
        return -1;
    }

    /* send requests and verify results */
    uint8_t write_buff[3] = {3 , 0, 0};
    uint8_t read_buff[2] = {0 , 0};
    pi_i2c_xfer_flags_e flag = PI_I2C_XFER_STOP | PI_I2C_XFER_START;
    int status;
    while(1)
    {
        // TODO use different write buffers and addresses for each master

        // Asynchronous tests
        write_buff[2]++; // increase the first written byte, so that we can tell if the master is working
        pi_task_t callback_tx, callback_rx;
        pi_task_callback(&callback_tx, __end_of_tx, &callback_tx);
        callback_tx.arg[2] = (int) &callback_rx;
        pi_task_callback(&callback_rx, __end_of_rx, &callback_rx);
        pi_i2c_write_async(master_dev0, write_buff, 3, flag, &callback_tx);

        // Synchronous tests
        write_buff[2]++; // increase the first written byte, so that we can tell if the master is working
        status = pi_i2c_write(master_dev1, write_buff, 3, flag);
        if (status != PI_OK)
        {
            printf("Write status: %d\n", status);
        }
        status = pi_i2c_read(master_dev1, read_buff, 2, flag);
        if (status != PI_OK)
        {
            printf("Read status: %d\n", status);
        }

        // wait asynchronous tests
        while (!done)
        {
            pi_yield();
        }
        if (async_error)
        {
            printf("Async error: %d\n", async_error);
        }
        done = 0;
        async_error = 0;
    }

    /* close and free */
    pi_i2c_close(master_dev0);
    pi_l2_free(master_dev0, sizeof(pi_device_t));

    pi_i2c_close(master_dev1);
    pi_l2_free(master_dev1, sizeof(pi_device_t));

    return 0;
}

/**
 * \brief launch the main test
 *
 * \warning hangs indefinitely
 */
int test_main(void)
{
    PRINTF("Application start\n");

    int ret = 0;

    /* launch slave */
    if(slave_launch())
    {
        PRINTF("Error while opening slave");
        ret = 1;
    }

#if MASTER_ENABLED
    /* launch the master */
    if(!ret && masters_launch())
    {
        PRINTF("Error while running master\n");
        ret = 1;
    }
#else
    pi_time_wait_us(1000 * 1000 * 600);
#endif

    /* close slave */
    if(!ret && slave_close())
    {
        PRINTF("Error while closing slave\n");
        ret = 1;
    }

    if(ret)
    {
        //TODO make a global error counter ? (atomic ?)
        // volatile should be sufficient since there is only one core
        printf("test returned with %d errors\n", ret);
    }
    pmsis_exit(ret);
    while(1);
    return 0;
}

int main(void)
{
    return pmsis_kickoff((void*) test_main);
}

