#ifndef UART_UARTDRIVER_H
#define UART_UARTDRIVER_H

#if __cplusplus < 201703L
#error "UartDriver requires C++17 or later"
#endif

#include <cstdint>
#include <cstddef>
#include <array>
#include "../drivers/dma_channel.h"

namespace uart {

class UartDriver {
public:
    UartDriver();
    ~UartDriver();

    UartDriver(const UartDriver&) = delete;
    UartDriver& operator=(const UartDriver&) = delete;

    void init(std::uint32_t baud_rate);

    void setBaudRate(std::uint32_t baud_rate);

    void flushRx();

    std::size_t read(std::uint8_t* buf, std::size_t len);

    bool isReadable() const;

    void write(const std::uint8_t* data, std::size_t len);

private:
    bool initialized_;

    static constexpr std::uint8_t kTXPin = 0;
    static constexpr std::uint8_t kRXPin = 1;

    bool dma_enabled_ = false;
    drivers::DmaChannel dma_rx_channel_;

    static constexpr std::size_t kDmaRxBufferSize = 1024;
    static constexpr std::size_t kDmaRxBufferMask = kDmaRxBufferSize - 1;
    static constexpr std::uint32_t kDmaTransferCount = 0xFFFFFFFF;

    std::array<std::uint8_t, kDmaRxBufferSize> dma_rx_buffer_;
    std::size_t dma_rx_read_pos_ = 0;

    void initDmaRx();
    std::size_t dmaWritePos() const;
    std::size_t dmaAvailable() const;
};

} // namespace uart

#endif // UART_UARTDRIVER_H