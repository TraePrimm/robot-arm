/**
 * as5047p.c
 *
 * AS5047P SPI frame (16 bits, see datasheet section "Interface Protocol"):
 *   bit15    = even parity over bits 14-0
 *   bit14    = R/W (1 = read, 0 = write)
 *   bits13-0 = register address (for a command frame) or data/error (reply)
 *
 * The part is pipelined: the data you request in frame N is clocked OUT
 * during frame N+1. So a read is: send command frame -> send a second
 * (NOP) frame -> the second frame's MISO data is your answer.
 *
 * ANGLECOM (0x3FFF) = angle with Dynamic Angle Error Compensation applied.
 * This is the one you want for a spinning application.
 */

#include "as5047p.h"

#define AS5047P_REG_NOP       0x0000
#define AS5047P_REG_ANGLECOM  0x3FFF
#define AS5047P_READ_BIT      0x4000

static void as5047p_cs_low(AS5047P_t *enc)  { HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET); }
static void as5047p_cs_high(AS5047P_t *enc) { HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET); }

static uint8_t as5047p_even_parity(uint16_t value)
{
    uint8_t parity = 0;
    for (int i = 0; i < 15; i++) {
        parity ^= (value >> i) & 0x01;
    }
    return parity;
}

static uint16_t as5047p_transfer(AS5047P_t *enc, uint16_t command)
{
    uint8_t tx[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
    uint8_t rx[2] = { 0 };

    as5047p_cs_low(enc);
    HAL_SPI_TransmitReceive(enc->hspi, tx, rx, 1, 10);
    as5047p_cs_high(enc);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

void AS5047P_Init(AS5047P_t *enc, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    enc->hspi = hspi;
    enc->cs_port = cs_port;
    enc->cs_pin = cs_pin;
    as5047p_cs_high(enc); // idle high
}

bool AS5047P_ReadAngleRaw(AS5047P_t *enc, uint16_t *angle_out)
{
    uint16_t cmd = AS5047P_REG_ANGLECOM | AS5047P_READ_BIT;
    cmd |= ((uint16_t)as5047p_even_parity(cmd)) << 15;

    /* First transfer sends the request; the reply to THIS request comes
     * back on the next transfer. Send a NOP to clock it out. */
    as5047p_transfer(enc, cmd);

    uint16_t nop = AS5047P_REG_NOP;
    nop |= ((uint16_t)as5047p_even_parity(nop)) << 15;
    uint16_t reply = as5047p_transfer(enc, nop);

    /* bit14 (EF, error flag) set means the sensor is reporting a fault
     * (e.g. magnet out of range) - caller should treat the reading as
     * suspect and check the ERRFL register. */
    bool error_flag = (reply >> 14) & 0x01;
    if (error_flag) {
        return false;
    }

    /* Parity check on the reply itself. */
    uint8_t expected_parity = as5047p_even_parity(reply & 0x7FFF);
    uint8_t received_parity = (reply >> 15) & 0x01;
    if (expected_parity != received_parity) {
        return false;
    }

    *angle_out = reply & 0x3FFF; // 14-bit angle
    return true;
}

bool AS5047P_ReadAngleDeg(AS5047P_t *enc, float *angle_deg_out)
{
    uint16_t raw;
    if (!AS5047P_ReadAngleRaw(enc, &raw)) {
        return false;
    }
    *angle_deg_out = ((float)raw / 16384.0f) * 360.0f;
    return true;
}