#include "UartDriver.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/structs/uart.h"
namespace uart {

UartDriver::UartDriver() : initialized_(false) {}

UartDriver::~UartDriver() {
    if (dma_enabled_) {
        dma_rx_channel_.release();
    }
    if (initialized_) {
        uart_deinit(uart0);
    }
}

void UartDriver::initDmaRx() {
    if (!dma_rx_channel_.claim()) {
        dma_enabled_ = false;
        return;
    }

    dma_rx_buffer_.fill(0);
    dma_rx_read_pos_ = 0;

    static_assert(alignof(UartDriver) >= kDmaRxBufferSize,
                  "UartDriver must be aligned to 1024 for DMA ring-buffer wrap");

    auto config = dma_rx_channel_.get_default_config();
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, DREQ_UART0_RX);
    channel_config_set_ring(&config, true, 10);

    const volatile void* uart_dr = &uart_get_hw(uart0)->dr;

    dma_rx_channel_.configure(
        config,
        dma_rx_buffer_.data(),
        uart_dr,
        kDmaTransferCount
    );

    dma_rx_channel_.start_transfer(true);
    dma_enabled_ = true;
}

std::size_t UartDriver::dmaWritePos() const {
    if (!dma_enabled_) return 0;

    dma_channel_hw_t* hw = dma_channel_hw_addr(dma_rx_channel_.channel_num());
    std::uint32_t remaining = hw->transfer_count;
    return (kDmaTransferCount - remaining) & kDmaRxBufferMask;
}

std::size_t UartDriver::dmaAvailable() const {
    if (!dma_enabled_) return 0;

    std::size_t write_pos = dmaWritePos();
    return (write_pos - dma_rx_read_pos_) & kDmaRxBufferMask;
}

void UartDriver::init(std::uint32_t baud_rate) {
    if (initialized_) return;

    uart_init(uart0, baud_rate);

    gpio_set_function(kTXPin, GPIO_FUNC_UART);
    gpio_set_function(kRXPin, GPIO_FUNC_UART);

    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    initDmaRx();

    initialized_ = true;
}

void UartDriver::setBaudRate(std::uint32_t baud_rate) {
    if (!initialized_) return;
    uart_set_baudrate(uart0, baud_rate);
}

void UartDriver::flushRx() {
    if (!initialized_) return;

    if (dma_enabled_) {
        dma_rx_read_pos_ = dmaWritePos();
        return;
    }

    while (uart_is_readable(uart0)) {
        uart_getc(uart0);
    }
}

std::size_t UartDriver::read(std::uint8_t* buf, std::size_t len) {
    if (!initialized_ || buf == nullptr || len == 0) return 0;

    if (dma_enabled_) {
        std::size_t avail = dmaAvailable();
        std::size_t to_read = (avail < len) ? avail : len;
        if (to_read == 0) return 0;

        for (std::size_t i = 0; i < to_read; ++i) {
            buf[i] = dma_rx_buffer_[(dma_rx_read_pos_ + i) & kDmaRxBufferMask];
        }
        dma_rx_read_pos_ = (dma_rx_read_pos_ + to_read) & kDmaRxBufferMask;
        return to_read;
    }

    std::size_t count = 0;
    while (count < len && uart_is_readable(uart0)) {
        buf[count++] = uart_getc(uart0);
    }
    return count;
}

bool UartDriver::isReadable() const {
    if (!initialized_) return false;

    if (dma_enabled_) {
        return dmaAvailable() > 0;
    }

    return uart_is_readable(uart0);
}

void UartDriver::write(const std::uint8_t* data, std::size_t len) {
    if (!initialized_ || data == nullptr || len == 0) return;
    uart_write_blocking(uart0, data, len);
}

} // namespace uart