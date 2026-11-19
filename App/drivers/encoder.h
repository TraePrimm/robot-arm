
/**
 * encoder.h
 *
 * Thin wrapper around the CircularBuffer AS5047P driver. joint.c only talks
 * to this file, so the third-party library can be swapped without touching
 * the joint code.
 *
 * Threading: call every Encoder_* function from ONE task (ControlLoop).
 * Other tasks read the cached fields (last_deg, last_diaagc, counters).
 */
#ifndef ENCODER_H
#define ENCODER_H
 
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "as5047p.h"
 
typedef struct {
    uint8_t          id;           /* 0..5, also the CS table index */
    AS5047P_Instance inst;         /* driver state */
 
    /* cached results, safe to read from other tasks (single writer) */
    volatile float    last_deg;    /* 0..360, last good reading */
    volatile uint16_t last_diaagc; /* last DIAAGC read (init or Encoder_ReadDiag) */
    volatile int16_t  last_err;    /* last driver error code, 0 = none */
    volatile uint32_t ok_count;
    volatile uint32_t err_count;
} Encoder_t;
 
/* Registers the CS pin, initializes the chip, and checks DIAAGC:
 * returns false if the chip does not answer or offset compensation (LF)
 * is not finished. */
bool Encoder_Init(Encoder_t *enc, uint8_t id, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);
 
/* Reads ANGLECOM (with the driver's reachability check). On success writes
 * 0..360 deg to *deg and returns true. On failure returns false, records
 * the error code, and clears the chip's error flag for the next read. */
bool Encoder_ReadDeg(Encoder_t *enc, float *deg);
 
/* Reads DIAAGC (LF / COF / MAGH / MAGL / AGC). For diagnostics at a low
 * rate, from the same task as Encoder_ReadDeg. */
bool Encoder_ReadDiag(Encoder_t *enc, uint16_t *diaagc);
 
/* DIAAGC bit helpers */
#define ENC_DIAG_AGC(d)   ((uint8_t)((d) & 0xFFu))
#define ENC_DIAG_LF(d)    (((d) >> 8) & 1u)
#define ENC_DIAG_COF(d)   (((d) >> 9) & 1u)
#define ENC_DIAG_MAGH(d)  (((d) >> 10) & 1u)
#define ENC_DIAG_MAGL(d)  (((d) >> 11) & 1u)
 
#endif /* ENCODER_H */
 