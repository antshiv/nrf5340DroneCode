#include "../../includes.h"
#include "spis_local.h"

const struct device *spi_slave_dev;
uint8_t slave_tx_buffer[2];
uint8_t slave_rx_buffer[2];

void spi_slave_init(void)
{
    spi_slave_dev = DEVICE_DT_GET(MY_SPI_SLAVE);
    if (!device_is_ready(spi_slave_dev))
    {
        printk("SPI slave device not ready!\n");
    }
}

int spi_slave_write_test_msg(void)
{
    static uint8_t counter = 0;

    const struct spi_buf s_tx_buf = {
        .buf = slave_tx_buffer,
        .len = sizeof(slave_tx_buffer)};
    const struct spi_buf_set s_tx = {
        .buffers = &s_tx_buf,
        .count = 1};

    struct spi_buf s_rx_buf = {
        .buf = slave_rx_buffer,
        .len = sizeof(slave_rx_buffer),
    };
    const struct spi_buf_set s_rx = {
        .buffers = &s_rx_buf,
        .count = 1};

    // Update the TX buffer with a rolling counter
    slave_tx_buffer[1] = counter++;
    printk("SPI SLAVE TX: 0x%.2x, 0x%.2x\n", slave_tx_buffer[0], slave_tx_buffer[1]);

    // Reset signal
    k_poll_signal_reset(&spi_slave_done_sig);

    // Start transaction
    int error = spi_transceive_async(spi_slave_dev, &spi_slave_cfg, &s_tx, &s_rx, &spi_slave_done_sig);
    if (error != 0)
    {
        printk("SPI slave transceive error: %i\n", error);
        return error;
    }
    return 0;
}

int spi_slave_check_for_message(void)
{
    int signaled, result;
    k_poll_signal_check(&spi_slave_done_sig, &signaled, &result);
    if (signaled != 0)
    {
        return 0;
    }
    else
        return -1;
}

void spi_slave_process_message(void)
{
    if (spi_slave_check_for_message() == 0)
    {
        // Print the last received data
        printk("SPI SLAVE RX: 0x%.2x, 0x%.2x\n", slave_rx_buffer[0], slave_rx_buffer[1]);

        // Prepare the next SPI slave transaction
        spi_slave_write_test_msg();
    }
    printk("SPI SLAVE RX: 0x%.2x, 0x%.2x\n", slave_rx_buffer[0], slave_rx_buffer[1]);
}