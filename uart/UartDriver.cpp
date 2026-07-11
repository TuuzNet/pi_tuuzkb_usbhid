#include "UartDriver.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include <cstring>

namespace uart {

UartDriver::UartDriver()
    : initialized_(false)
    , dma_channel_(-1)
    , last_read_pos_(0) {
    dma_buf_.fill(0);
}

UartDriver::~UartDriver() {
    if (initialized_) {
        if (dma_channel_ >= 0) {
            dma_channel_abort(dma_channel_);
            dma_channel_unclaim(dma_channel_);
        }
        uart_deinit(uart0);
    }
}

void UartDriver::init(std::uint32_t baud_rate) {
    if (initialized_) return;

    uart_init(uart0, baud_rate);

    gpio_set_function(kTXPin, GPIO_FUNC_UART);
    gpio_set_function(kRXPin, GPIO_FUNC_UART);

    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    dma_channel_ = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(dma_channel_);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_UART0_RX);
    channel_config_set_ring(&cfg, true, 8);
    channel_config_set_chain_to(&cfg, dma_channel_);

    dma_channel_configure(
        dma_channel_,
        &cfg,
        dma_buf_.data(),
        &uart_get_hw(uart0)->dr,
        kDmaBufSize,
        false
    );

    dma_channel_start(dma_channel_);

    initialized_ = true;
}

void UartDriver::setBaudRate(std::uint32_t baud_rate) {
    if (!initialized_) return;
    uart_set_baudrate(uart0, baud_rate);
}

void UartDriver::flushRx() {
    if (!initialized_) return;
    last_read_pos_ = getWritePos();
}

std::size_t UartDriver::getWritePos() const {
    if (!initialized_) return 0;
    std::uint32_t remaining = dma_channel_hw_addr(dma_channel_)->transfer_count;
    return (kDmaBufSize - remaining) & kDmaBufMask;
}

std::size_t UartDriver::read(std::uint8_t* buf, std::size_t len) {
    if (!initialized_ || buf == nullptr || len == 0) return 0;

    std::size_t write_pos = getWritePos();
    std::size_t available = (write_pos - last_read_pos_) & kDmaBufMask;

    if (available == 0) return 0;

    std::size_t count = (available < len) ? available : len;

    if (last_read_pos_ + count <= kDmaBufSize) {
        std::memcpy(buf, &dma_buf_[last_read_pos_], count);
    } else {
        std::size_t first_part = kDmaBufSize - last_read_pos_;
        std::memcpy(buf, &dma_buf_[last_read_pos_], first_part);
        std::memcpy(buf + first_part, &dma_buf_[0], count - first_part);
    }

    last_read_pos_ = (last_read_pos_ + count) & kDmaBufMask;
    return count;
}

bool UartDriver::isReadable() const {
    if (!initialized_) return false;
    std::size_t write_pos = getWritePos();
    return ((write_pos - last_read_pos_) & kDmaBufMask) != 0;
}

void UartDriver::write(const std::uint8_t* data, std::size_t len) {
    if (!initialized_ || data == nullptr || len == 0) return;
    uart_write_blocking(uart0, data, len);
}

} // namespace uart