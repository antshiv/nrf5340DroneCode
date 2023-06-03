#include "../../includes.h"


#define MY_SPI_SLAVE DT_NODELABEL(my_spi_slave)

extern uint8_t *lave_tx_buffer[2];
extern uint8_t slave_rx_buffer[2];

// SPI slave functionality
extern const struct device *spi_slave_dev;
static struct k_poll_signal spi_slave_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_slave_done_sig);

static const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
	.frequency = 4000000,
	.slave = 0,
};


void pwm_check_ready();
void spi_slave_init(void);
int spi_slave_check_for_message(void);
int spi_slave_write_test_msg(void);
void spi_slave_process_message(void);