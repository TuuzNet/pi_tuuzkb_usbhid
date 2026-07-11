#ifndef UART_UARTDRIVER_H
#define UART_UARTDRIVER_H

#if __cplusplus < 201703L
#error "UartDriver requires C++17 or later"
#endif

#include <cstdint>
#include <cstddef>
#include <array>

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
    static constexpr std::size_t kDmaBufSize = 256;
    static constexpr std::size_t kDmaBufMask = kDmaBufSize - 1;

    bool initialized_;
    int dma_channel_;
    std::array<std::uint8_t, kDmaBufSize> dma_buf_;
    std::size_t last_read_pos_;

    static constexpr std::uint8_t kTXPin = 0;
    static constexpr std::uint8_t kRXPin = 1;

    std::size_t getWritePos() const;
};

} // namespace uart

#endif // UART_UARTDRIVER_H