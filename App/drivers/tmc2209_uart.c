/* tmc2209_uart.c: Layer B.
 *
 * Verified against this project's HAL source:
 *  - HAL_HalfDuplex_EnableTransmitter/Receiver each clear TE and RE, then set
 *    one of them, so the receiver is off while we transmit (no echo).
 *  - HAL_UART_Transmit returns only after the TC flag, i.e. the last stop bit
 *    has left the wire.
 *
 * Not verified yet: the driver's reply delay (SLAVECONF SENDDELAY). The gap
 * between end-of-transmit and enabling the receiver must stay far below it.
 *
 * NOT thread-safe: call from one task only until Layer C adds a bus mutex. A
 * task switch inside that gap could make us miss the start of the reply. */
#include "tmc2209_uart.h"

#define TMC_UART_TIMEOUT_MS  5   /* 8 bytes at 115200 is ~0.7 ms; HAL tick is 1 ms */

/* Drop anything sitting in the receive data register and clear latched error
 * flags. FIFO mode is disabled in CubeMX, so there is one data register. */
static void tmc_uart_rx_reset(UART_HandleTypeDef *h)
{
    __HAL_UART_SEND_REQ(h, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(h, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
}

tmc_io_status_t tmc_uart_write(UART_HandleTypeDef *h, const uint8_t frame[8])
{
    if (HAL_HalfDuplex_EnableTransmitter(h) != HAL_OK)                  return TMC_IO_TX_FAIL;
    if (HAL_UART_Transmit(h, frame, 8, TMC_UART_TIMEOUT_MS) != HAL_OK)  return TMC_IO_TX_FAIL;
    return TMC_IO_OK;
}

tmc_io_status_t tmc_uart_read(UART_HandleTypeDef *h, const uint8_t req[4],
                              uint8_t reply[8])
{
    if (HAL_HalfDuplex_EnableTransmitter(h) != HAL_OK)                 return TMC_IO_TX_FAIL;
    if (HAL_UART_Transmit(h, req, 4, TMC_UART_TIMEOUT_MS) != HAL_OK)   return TMC_IO_TX_FAIL;

    /* Flush while the receiver is still off so stale bytes and old error flags
     * can't be mistaken for the reply, then listen immediately. */
    tmc_uart_rx_reset(h);
    if (HAL_HalfDuplex_EnableReceiver(h) != HAL_OK)                    return TMC_IO_TX_FAIL;

    if (HAL_UART_Receive(h, reply, 8, TMC_UART_TIMEOUT_MS) != HAL_OK)  return TMC_IO_RX_TIMEOUT;
    return TMC_IO_OK;
}