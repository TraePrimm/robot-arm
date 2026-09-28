/* tmc2209_uart.h: Layer B, single-wire UART transport for the TMC2209.
 * Moves raw bytes only. Knows nothing about registers or CRC. */
#ifndef TMC2209_UART_H
#define TMC2209_UART_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    TMC_IO_OK = 0,
    TMC_IO_TX_FAIL,      /* direction switch or transmit failed */
    TMC_IO_RX_TIMEOUT,   /* request sent, no complete 8-byte reply in time */
} tmc_io_status_t;

/* Send an 8-byte write frame. No reply exists; confirm via IFCNT in Layer C. */
tmc_io_status_t tmc_uart_write(UART_HandleTypeDef *h, const uint8_t frame[8]);

/* Send a 4-byte read request, then listen for the 8-byte reply. */
tmc_io_status_t tmc_uart_read(UART_HandleTypeDef *h, const uint8_t req[4],
                              uint8_t reply[8]);

#endif /* TMC2209_UART_H */