/* tmc2209_reg.c: Layer C. See tmc2209_reg.h. */
#include "tmc2209_reg.h"
#include "tmc2209_frame.h"

#define TMC_READ_TRIES   3
#define TMC_WRITE_TRIES  3

void tmc_dev_init(tmc_dev_t *d, UART_HandleTypeDef *huart, uint8_t addr)
{
    d->huart = huart;
    d->addr = addr;
    d->ifcnt = 0;
    d->ifcnt_valid = false;   /* unknown until we read it from the chip */
}

static tmc_status_t read_once(tmc_dev_t *d, uint8_t reg, uint32_t *val)
{
    uint8_t req[4], reply[8];
    tmc2209_build_read(d->addr, reg, req);

    if (tmc_uart_read(d->huart, req, reply) != TMC_IO_OK)
        return TMC_ERR_IO;
    if (tmc2209_parse_reply(reply, reg, val) != TMC_PARSE_OK)
        return TMC_ERR_PARSE;
    return TMC_OK;
}

tmc_status_t tmc_read_reg(tmc_dev_t *d, uint8_t reg, uint32_t *val)
{
    tmc_status_t st = TMC_ERR_IO;
    for (int i = 0; i < TMC_READ_TRIES; i++) {
        st = read_once(d, reg, val);
        if (st == TMC_OK) break;
    }
    return st;
}

tmc_status_t tmc_write_reg(tmc_dev_t *d, uint8_t reg, uint32_t val)
{
    uint8_t frame[8];
    tmc2209_build_write(d->addr, reg, val, frame);

    tmc_status_t st = TMC_ERR_NOT_CONFIRMED;

    for (int attempt = 0; attempt < TMC_WRITE_TRIES; attempt++) {
        uint32_t cnt;

        /* 1. Make sure we know the chip's current counter. */
        if (!d->ifcnt_valid) {
            st = tmc_read_reg(d, TMC_REG_IFCNT, &cnt);
            if (st != TMC_OK) continue;
            d->ifcnt = (uint8_t)cnt;
            d->ifcnt_valid = true;
        }

        /* 2. Send the write. If this fails we don't know what the chip saw. */
        if (tmc_uart_write(d->huart, frame) != TMC_IO_OK) {
            d->ifcnt_valid = false;
            st = TMC_ERR_IO;
            continue;
        }

        /* 3. Ask the chip how many writes it has accepted now. */
        st = tmc_read_reg(d, TMC_REG_IFCNT, &cnt);
        if (st != TMC_OK) {
            d->ifcnt_valid = false;   /* unknown whether it landed: re-seed next time */
            continue;
        }

        uint8_t now = (uint8_t)cnt;
        if (now == (uint8_t)(d->ifcnt + 1)) {   /* cast handles 255 -> 0 wrap */
            d->ifcnt = now;
            return TMC_OK;
        }

        /* Counter didn't move by exactly 1: the write was lost. Resync to the
         * chip's real value and retry (all our config writes are idempotent). */
        d->ifcnt = now;
        st = TMC_ERR_NOT_CONFIRMED;
    }
    return st;
}