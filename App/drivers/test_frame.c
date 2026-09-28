/* test_frame.c: laptop-only tests for the TMC2209 framing layer */
#include <stdio.h>
#include <stdint.h>
#include "tmc2209_frame.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

/* Build a valid driver->master reply by hand (independent of build_write). */
static void make_reply(uint8_t reg, uint32_t val, uint8_t r[8])
{
    r[0] = TMC_SYNC;
    r[1] = TMC_MASTER_ADDR;
    r[2] = reg;
    r[3] = (uint8_t)(val >> 24);
    r[4] = (uint8_t)(val >> 16);
    r[5] = (uint8_t)(val >> 8);
    r[6] = (uint8_t)val;
    r[7] = tmc2209_crc8(r, 7);
}

static void test_build_frames(void)
{
    uint8_t w[8], rd[4];

    /* Endianness: 0x01020304 must go out as 01 02 03 04, MSB first. */
    tmc2209_build_write(2, 0x10, 0x01020304, w);
    CHECK(w[0] == 0x05 && w[1] == 2);
    CHECK(w[2] == 0x90);                       /* 0x10 | 0x80 */
    CHECK(w[3] == 0x01 && w[4] == 0x02 && w[5] == 0x03 && w[6] == 0x04);
    CHECK(w[7] == tmc2209_crc8(w, 7));

    tmc2209_build_read(1, 0x02, rd);
    CHECK(rd[0] == 0x05 && rd[1] == 1 && rd[2] == 0x02);
    CHECK(rd[3] == tmc2209_crc8(rd, 3));

    /* A reg with the top bit set must not turn a read into a write. */
    tmc2209_build_read(0, 0x82, rd);
    CHECK(rd[2] == 0x02);
}

static void test_round_trip(void)
{
    uint8_t r[8];
    uint32_t v = 0;
    make_reply(0x06, 0x21000000, r);
    CHECK(tmc2209_parse_reply(r, 0x06, &v) == TMC_PARSE_OK);
    CHECK(v == 0x21000000);
}

static void test_corruption(void)
{
    /* Flip every single bit of a valid frame; the parser must reject all of them. */
    for (int byte = 0; byte < 8; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t r[8];
            uint32_t v = 0xDEADBEEF;             /* sentinel: must stay untouched */
            make_reply(0x06, 0x12345678, r);
            r[byte] ^= (uint8_t)(1u << bit);

            tmc_parse_status_t s = tmc2209_parse_reply(r, 0x06, &v);
            CHECK(s != TMC_PARSE_OK);
            CHECK(v == 0xDEADBEEF);
            /* Byte 0 is the sync byte, checked first. Everything else is caught by CRC. */
            CHECK(s == (byte == 0 ? TMC_PARSE_BAD_SYNC : TMC_PARSE_BAD_CRC));
        }
    }
}

static void test_valid_crc_wrong_content(void)
{
    uint8_t r[8];
    uint32_t v = 0;

    /* Good CRC, but the reply is for a different register than we asked for. */
    make_reply(0x02, 0, r);
    CHECK(tmc2209_parse_reply(r, 0x06, &v) == TMC_PARSE_BAD_REG);

    /* Good CRC, but byte 1 isn't the master address 0xFF. */
    make_reply(0x06, 0, r);
    r[1] = 0x00;
    r[7] = tmc2209_crc8(r, 7);                   /* re-seal so only byte 1 is wrong */
    CHECK(tmc2209_parse_reply(r, 0x06, &v) == TMC_PARSE_BAD_MASTER_ADDR);
}

int main(void)
{
    test_build_frames();
    test_round_trip();
    test_corruption();
    test_valid_crc_wrong_content();

    /* TODO: paste a worked example frame from the TMC2209 datasheet's UART section
     * and check that tmc2209_crc8() reproduces its CRC byte. */

    if (failures == 0) printf("all tests passed\n");
    return failures ? 1 : 0;
}