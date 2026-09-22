/**
 * tmc2209.h
 *
 * Driver for the TMC2209 stepper driver's UART interface.
 * Knows nothing about "joints" or "the arm" — purely register read/write
 * over a half-duplex UART bus, plus a couple of convenience wrappers.
 *
 * Up to 4 TMC2209s can share one UART bus (addresses 0-3, set by the
 * driver's MS1/MS2 pins). Call tmc2209_init() once per motor with the
 * UART handle it lives on and its hardware address.
 */
#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"   // adjust if your HAL family header differs

/* ---- Commonly used TMC2209 register addresses (see datasheet Table 5) ---- */
#define TMC2209_REG_GCONF       0x00
#define TMC2209_REG_GSTAT       0x01
#define TMC2209_REG_IFCNT       0x02
#define TMC2209_REG_SLAVECONF   0x03
#define TMC2209_REG_IOIN        0x06
#define TMC2209_REG_IHOLD_IRUN  0x10
#define TMC2209_REG_TPOWERDOWN  0x11
#define TMC2209_REG_TSTEP       0x12
#define TMC2209_REG_VACTUAL     0x22   // unused in STEP/DIR mode, kept for reference
#define TMC2209_REG_MSCNT       0x6A
#define TMC2209_REG_CHOPCONF    0x6C
#define TMC2209_REG_DRV_STATUS  0x6F
#define TMC2209_REG_PWMCONF     0x70

typedef struct {
    UART_HandleTypeDef *huart;  // which physical UART bus this driver is on
    uint8_t address;            // 0-3, set by MS1/MS2 wiring
    uint32_t last_ifcnt;        // used to verify a write actually landed
} TMC2209_t;

/* Init: just stores the handle/address, does NOT touch the part yet. */
void TMC2209_Init(TMC2209_t *drv, UART_HandleTypeDef *huart, uint8_t address);

/* Bring the driver into a known-good state: clear GSTAT, set a sane
 * CHOPCONF (spreadCycle, microstepping), enable UART-controlled current. */
bool TMC2209_Configure(TMC2209_t *drv, uint16_t microsteps);

/* Raw register access. Returns true on success (write verified via IFCNT,
 * read verified via CRC match). */
bool TMC2209_WriteRegister(TMC2209_t *drv, uint8_t reg_addr, uint32_t data);
bool TMC2209_ReadRegister(TMC2209_t *drv, uint8_t reg_addr, uint32_t *data_out);

/* Convenience wrappers */
bool TMC2209_SetRunCurrent(TMC2209_t *drv, uint8_t run_current_0_31, uint8_t hold_current_0_31);
bool TMC2209_GetStatus(TMC2209_t *drv, uint32_t *drv_status_out);
bool TMC2209_IsConnected(TMC2209_t *drv); // reads IOIN, checks it's non-garbage

#endif // TMC2209_H