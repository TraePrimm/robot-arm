/* tmc2209_frame.c: TMC2209 UART framing. No HAL, no hardware. */
#include "tmc2209_frame.h"

uint8_t tmc2209_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            b >>= 1;
        }
    }
    return crc;
}

void tmc2209_build_write(uint8_t addr, uint8_t reg, uint32_t val, uint8_t out[8])
{
    out[0] = TMC_SYNC;
    out[1] = addr;
    out[2] = reg | 0x80;              /* top bit set = write */
    out[3] = (uint8_t)(val >> 24);    /* MSB first, regardless of CPU endianness */
    out[4] = (uint8_t)(val >> 16);
    out[5] = (uint8_t)(val >> 8);
    out[6] = (uint8_t)(val);
    out[7] = tmc2209_crc8(out, 7);
}

void tmc2209_build_read(uint8_t addr, uint8_t reg, uint8_t out[4])
{
    out[0] = TMC_SYNC;
    out[1] = addr;
    out[2] = reg & 0x7F;              /* top bit clear = read */
    out[3] = tmc2209_crc8(out, 3);
}

tmc_parse_status_t tmc2209_parse_reply(const uint8_t in[8], uint8_t expected_reg,
                                       uint32_t *val_out)
{
    if (in[0] != TMC_SYNC)                  return TMC_PARSE_BAD_SYNC;
    if (tmc2209_crc8(in, 7) != in[7])       return TMC_PARSE_BAD_CRC;
    if (in[1] != TMC_MASTER_ADDR)           return TMC_PARSE_BAD_MASTER_ADDR;
    if (in[2] != (expected_reg & 0x7F))     return TMC_PARSE_BAD_REG;

    *val_out = ((uint32_t)in[3] << 24) | ((uint32_t)in[4] << 16) |
               ((uint32_t)in[5] << 8)  |  (uint32_t)in[6];
    return TMC_PARSE_OK;
}