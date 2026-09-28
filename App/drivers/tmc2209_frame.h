/* tmc2209_frame.h: pure framing, no HAL */
#ifndef TMC2209_FRAME_H
#define TMC2209_FRAME_H
#include <stdint.h>

#define TMC_SYNC         0x05
#define TMC_MASTER_ADDR  0xFF   /* byte 1 of every reply */

typedef enum {
    TMC_PARSE_OK = 0,
    TMC_PARSE_BAD_SYNC,
    TMC_PARSE_BAD_CRC,
    TMC_PARSE_BAD_MASTER_ADDR,
    TMC_PARSE_BAD_REG,
} tmc_parse_status_t;

uint8_t tmc2209_crc8(const uint8_t *data, uint8_t len);
void    tmc2209_build_write(uint8_t addr, uint8_t reg, uint32_t val, uint8_t out[8]);
void    tmc2209_build_read (uint8_t addr, uint8_t reg, uint8_t out[4]);
tmc_parse_status_t tmc2209_parse_reply(const uint8_t in[8], uint8_t expected_reg,
                                       uint32_t *val_out);
#endif