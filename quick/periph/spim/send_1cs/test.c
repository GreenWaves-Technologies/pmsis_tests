#include <stdio.h>
#include <pmsis.h>
#include <stdint.h>

#if !defined(SYNC_CS_AUTO) && !defined(ASYNC_CS_AUTO) && !defined(SYNC_CS_KEEP) && !defined(ASYNC_CS_KEEP)
#define SYNC_CS_AUTO 1
#endif

#define TOTAL_SIZE (8192)
//#define TOTAL_SIZE (8192*8 + 256)
#define TOTAL_SIZE_RTL (1024)

#define NB_BUFFERS 4



static inline void get_info(int *buffer_size)
{
#if !defined( ARCHI_PLATFORM_RTL)
  *buffer_size = TOTAL_SIZE;
#else
  if (rt_platform() == ARCHI_PLATFORM_RTL)
  {
    *buffer_size = TOTAL_SIZE_RTL;
  }
  else
  {
    *buffer_size = TOTAL_SIZE;
  }
#endif
}



PI_L2 int32_t cmd_buffer[4][2];
PI_L2 int32_t rx_cmd_buffer[4][2];

PI_L2 uint8_t *tx_buffer;

PI_L2 uint8_t *rx_buffer;

static pi_task_t buf_event[NB_BUFFERS];
static pi_task_t rx_buf_event[NB_BUFFERS];
static pi_task_t cmd_event[NB_BUFFERS];
static pi_task_t rx_cmd_event[NB_BUFFERS];


static void set_spim_verif_command(struct pi_device *spim, int cmd, int addr, int size, int32_t *cmd_buffer, pi_task_t *task)
{
  cmd_buffer[0] = (cmd << 24) | (size*8);
  cmd_buffer[1] = addr;
  if (task)
    pi_spi_send_async(spim, cmd_buffer, 8*8, PI_SPI_CS_AUTO, task);
  else
    pi_spi_send(spim, cmd_buffer, 8*8, PI_SPI_CS_AUTO);
}


static int test_entry()
{
  struct pi_spi_conf conf;
  struct pi_device spim;

  int total_size;
  get_info(&total_size);
  int buffer_size = total_size / NB_BUFFERS;

  pi_spi_conf_init(&conf);

  conf.wordsize = PI_SPI_WORDSIZE_32;
  conf.big_endian = 1;
  conf.max_baudrate = 10000000;
  conf.polarity = 0;
  conf.phase = 0;
  conf.itf = SPIM_ITF;
  conf.cs = SPIM_CS;

  pi_open_from_conf(&spim, &conf);

  if (pi_spi_open(&spim))
    return -1;

  tx_buffer = pmsis_l2_malloc(total_size);
  if (tx_buffer == NULL) return -1;

  rx_buffer = pmsis_l2_malloc(total_size);
  if (rx_buffer == NULL) return -1;

  for (int i=0; i<total_size; i++)
  {
    tx_buffer[i] = i;
  }


#if defined(SYNC_CS_AUTO)

  for (int i=0; i<NB_BUFFERS; i++)
  {
    set_spim_verif_command(&spim, 0x1, buffer_size*i, buffer_size, cmd_buffer[i], NULL);
    pi_spi_send(&spim, tx_buffer+buffer_size*i, buffer_size*8, PI_SPI_CS_AUTO);  

    set_spim_verif_command(&spim, 0x2, buffer_size*i, buffer_size, cmd_buffer[i], NULL);
    pi_spi_receive(&spim, rx_buffer+buffer_size*i, buffer_size*8, PI_SPI_CS_AUTO);
  }

#elif defined(ASYNC_CS_AUTO)

  for (int i=0; i<NB_BUFFERS; i++)
  {
    set_spim_verif_command(&spim, 0x1, buffer_size*i, buffer_size, cmd_buffer[i], pi_task_block(&cmd_event[i]));
    pi_spi_send_async(&spim, tx_buffer+buffer_size*i, buffer_size*8, PI_SPI_CS_AUTO, pi_task_block(&buf_event[i]));
  }

  for (int i=0; i<NB_BUFFERS; i++)
  {
    set_spim_verif_command(&spim, 0x2, buffer_size*i, buffer_size, rx_cmd_buffer[i], pi_task_block(&rx_cmd_event[i]));
    pi_spi_receive_async(&spim, rx_buffer+buffer_size*i, buffer_size*8, PI_SPI_CS_AUTO, pi_task_block(&rx_buf_event[i]));
  }

  for (int i=0; i<NB_BUFFERS; i++)
  {
    pi_task_wait_on(&cmd_event[i]);
    pi_task_wait_on(&buf_event[i]);
    pi_task_wait_on(&rx_cmd_event[i]);
    pi_task_wait_on(&rx_buf_event[i]);
  }

#elif defined(SYNC_CS_KEEP)

  set_spim_verif_command(&spim, 0x1, 0, total_size, cmd_buffer[0], NULL);

  for (int i=0; i<NB_BUFFERS; i++)
  {
    int mode = i == NB_BUFFERS - 1 ? PI_SPI_CS_AUTO : PI_SPI_CS_KEEP;
    pi_spi_send(&spim, tx_buffer+buffer_size*i, buffer_size*8, mode);
  }

  set_spim_verif_command(&spim, 0x2, 0, total_size, cmd_buffer[0], NULL);

  for (int i=0; i<NB_BUFFERS; i++)
  {
    int mode = i == NB_BUFFERS - 1 ? PI_SPI_CS_AUTO : PI_SPI_CS_KEEP;
    pi_spi_receive(&spim, rx_buffer+buffer_size*i, buffer_size*8, mode);
  }

#elif defined(ASYNC_CS_KEEP)

  set_spim_verif_command(&spim, 0x1, 0, total_size, cmd_buffer[0], NULL);

  for (int i=0; i<NB_BUFFERS; i++)
  {
    int mode = i == NB_BUFFERS - 1 ? PI_SPI_CS_AUTO : PI_SPI_CS_KEEP;
    pi_spi_send_async(&spim, tx_buffer+buffer_size*i, buffer_size*8, mode, pi_task_block(&buf_event[i]));
  }

  set_spim_verif_command(&spim, 0x2, 0, total_size, cmd_buffer[0], pi_task_block(&cmd_event[0]));

  for (int i=0; i<NB_BUFFERS; i++)
  {
    int mode = i == NB_BUFFERS - 1 ? PI_SPI_CS_AUTO : PI_SPI_CS_KEEP;
    pi_spi_receive_async(&spim, rx_buffer+buffer_size*i, buffer_size*8, mode, pi_task_block(&rx_buf_event[i]));
  }

  pi_task_wait_on(&cmd_event[0]);

  for (int i=0; i<NB_BUFFERS; i++)
  {
    pi_task_wait_on(&buf_event[i]);
    pi_task_wait_on(&rx_buf_event[i]);
  }

#endif



  int error = 0;
  for (int i=0; i<total_size; i++)
  {
    //printf("%d: @%p: %x @%p: %x\n", i, &rx_buffer[i], rx_buffer[i], &tx_buffer[i], tx_buffer[i]);
    if (rx_buffer[i] != tx_buffer[i])
    {
      if (error == 0)
        printf("First error at index %d, expected 0x%x, got 0x%x at %p\n", i, tx_buffer[i], rx_buffer[i], &rx_buffer[i]);
      error++;
      return -1;
    }
  }

  if (error)
  {
    printf("Got %d errors\n", error);
  }
  else
  {
    printf("Test success\n");
  }
  pi_spi_close(&spim);
  return error;
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
