/* tmc2209_reg.h: Layer C. Register read/write on top of framing (A) and UART (B).
 *
 * - tmc_read_reg:  sends a read request, validates CRC + register echo, retries.
 * - tmc_write_reg: sends a write, then CONFIRMS it landed by checking that the
 *                  chip's IFCNT counter went up by exactly 1. Retries if not.
 *
 * NOT thread-safe: call from ONE task per UART bus (huart1 / huart2).
 * Add a per-bus mutex before two tasks share a bus. */
#ifndef TMC2209_REG_H
#define TMC2209_REG_H

#include <stdint.h>
#include <stdbool.h>
#include "tmc2209_uart.h"

/* Register addresses used so far */
#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IFCNT       0x02
#define TMC_REG_SLAVECONF   0x03
#define TMC_REG_IOIN        0x06
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_CHOPCONF    0x6C

typedef enum {
    TMC_OK = 0,
    TMC_ERR_IO,             /* wire problem: TX failed or reply timed out */
    TMC_ERR_PARSE,          /* reply arrived but bad sync/CRC/register */
    TMC_ERR_NOT_CONFIRMED,  /* write sent, but IFCNT never showed it landed */
    TMC_ERR_BAD_ARG,        /* caller passed an invalid value */
    TMC_ERR_WRONG_CHIP      /* IOIN version byte is not 0x21 */
} tmc_status_t;

/* One of these per physical driver chip. */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  addr;          /* 0-3, set by the chip's MS1/MS2 pins */
    uint8_t  ifcnt;         /* last known value of the chip's IFCNT */
    bool     ifcnt_valid;   /* false => re-read IFCNT before trusting 'ifcnt' */
} tmc_dev_t;

void         tmc_dev_init (tmc_dev_t *d, UART_HandleTypeDef *huart, uint8_t addr);
tmc_status_t tmc_read_reg (tmc_dev_t *d, uint8_t reg, uint32_t *val);
tmc_status_t tmc_write_reg(tmc_dev_t *d, uint8_t reg, uint32_t val);

#endif /* TMC2209_REG_H */