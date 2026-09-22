/**
 * as5047p.h
 *
 * Driver for the AS5047P magnetic rotary encoder's SPI interface.
 * Knows nothing about joints/gear ratios — returns raw sensor-shaft angle.
 */
#ifndef AS5047P_H
#define AS5047P_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} AS5047P_t;

void AS5047P_Init(AS5047P_t *enc, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Returns raw 14-bit angle (0-16383) from the ANGLECOM register (dynamic
 * angle error compensated output). Returns false if the parity/error
 * checks fail (retry once before trusting a failure). */
bool AS5047P_ReadAngleRaw(AS5047P_t *enc, uint16_t *angle_out);

/* Convenience: raw angle converted to degrees (0-360). */
bool AS5047P_ReadAngleDeg(AS5047P_t *enc, float *angle_deg_out);

#endif // AS5047P_H