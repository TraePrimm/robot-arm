
/**
 * encoder.c - see encoder.h
 */
#include "encoder.h"
 
/* Provided by our as5047p_hal.c (not part of the upstream header). */
uint8_t AS5047P_HAL_Register(uint16_t id, SPI_HandleTypeDef *spi,
                             GPIO_TypeDef *port, uint16_t pin);
 
/* The driver latches its FIRST error until acknowledged. After a failed read
 * we record the code and acknowledge it; ErrorAck also reads ERRFL, which
 * clears the chip's EF flag (otherwise every later reply carries EF and
 * fails as a "framing error"). If the ack itself fails (bus still bad), the
 * latched code is cleared by hand so the next read starts clean. */
static void encoder_handle_failure(Encoder_t *enc)
{
    enc->err_count++;
    if (AS5047P_ErrorPending(&enc->inst)) {
        enc->last_err = AS5047P_GetError(&enc->inst).errorCode;
        if (AS5047P_ErrorAck(&enc->inst) != 0) {
            enc->inst.error.errorCode = 0;
        }
    }
}
 
/* Success is judged by the return value. The driver can latch an error from
 * its discarded first frame even when the returned data is valid; clear it
 * without extra SPI traffic. */
static void encoder_handle_success(Encoder_t *enc)
{
    enc->ok_count++;
    enc->inst.error.errorCode = 0;
}
 
static bool encoder_read_reg(Encoder_t *enc, uint16_t reg, uint16_t *out)
{
    AS5047P_Result r = AS5047P_ReadRegister(&enc->inst, reg, AS5047P_OPT_ENABLED);
    if (r < 0) {
        encoder_handle_failure(enc);
        return false;
    }
    encoder_handle_success(enc);
    *out = (uint16_t)r;
    return true;
}
 
bool Encoder_Init(Encoder_t *enc, uint8_t id, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    enc->id = id;
    enc->last_deg = 0.0f;
    enc->last_diaagc = 0;
    enc->last_err = 0;
    enc->ok_count = 0;
    enc->err_count = 0;
 
    if (AS5047P_HAL_Register(id, spi, cs_port, cs_pin) != 0) {
        enc->last_err = 30;   /* same code the driver uses for HAL init failure */
        return false;
    }
 
    if (AS5047P_Init(&enc->inst, id) != 0) {
        encoder_handle_failure(enc);
        return false;
    }
 
    /* A stuck or unpowered chip can fail in ways parity cannot catch.
     * Require a sane DIAAGC: offset compensation finished (LF = 1). */
    uint16_t diag;
    if (!encoder_read_reg(enc, AS5047P_DIAAGC, &diag)) {
        return false;
    }
    enc->last_diaagc = diag;
    if (!ENC_DIAG_LF(diag)) {
        return false;
    }
 
    float deg;
    return Encoder_ReadDeg(enc, &deg);
}
 
bool Encoder_ReadDeg(Encoder_t *enc, float *deg)
{
    uint16_t raw;
    if (!encoder_read_reg(enc, AS5047P_ANGLECOM, &raw)) {
        return false;
    }
    float d = (float)(raw & 0x3FFFu) * (360.0f / 16384.0f);
    enc->last_deg = d;
    *deg = d;
    return true;
}
 
bool Encoder_ReadDiag(Encoder_t *enc, uint16_t *diaagc)
{
    uint16_t d;
    if (!encoder_read_reg(enc, AS5047P_DIAAGC, &d)) {
        return false;
    }
    enc->last_diaagc = d;
    *diaagc = d;
    return true;
}
 