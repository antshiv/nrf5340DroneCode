#include "../../includes.h"
#include "spim_local.h"

const struct device *spi_dev;
struct spi_cs_control spim_cs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
    .delay = 0,
};

const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .frequency = 1000000,
    .slave = 0,
    .cs = &spim_cs,
};

void spi_init(void)
{
    spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
    if (!device_is_ready(spi_dev))
    {
        printk("SPI master device not ready!\n");
    }
    if (!device_is_ready(spim_cs.gpio.port))
    {
        printk("SPI master chip select device not ready!\n");
    }
}

int spi_write_test_msg(void)
{
    static uint8_t counter = 0;
    static uint8_t tx_buffer[2];
    static uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer)};
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1};

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1};

    // Update the TX buffer with a rolling counter
    // tx_buffer[0] = counter++;
    tx_buffer[0] = 0x02;
    tx_buffer[1] = 0x02;
    printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

    // Reset signal
    k_poll_signal_reset(&spi_done_sig);

    // Start transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
    if (error != 0)
    {
        printk("SPI transceive error: %i\n", error);
        return error;
    }

    // Wait for the done signal to be raised and log the rx buffer
    int spi_signaled, spi_result;
    do
    {
        k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
    } while (spi_signaled == 0);
    printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
    return 0;
}
