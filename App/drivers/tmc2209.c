/**
 * tmc2209.c
 *
 * UART datagram formats (see TMC2209 datasheet section 5):
 *
 * Write (8 bytes, master -> driver):
 *   [0] sync = 0x05
 *   [1] slave address
 *   [2] register address | 0x80   (write bit)
 *   [3..6] 32-bit data, MSB first
 *   [7] CRC8
 *
 * Read request (4 bytes, master -> driver):
 *   [0] sync = 0x05
 *   [1] slave address
 *   [2] register address          (write bit = 0)
 *   [3] CRC8
 *
 * Read reply (8 bytes, driver -> master, arrives on the same half-duplex
 * line, so the driver's own TX echoes back before the reply):
 *   [0] sync = 0x05
 *   [1] master address = 0xFF
 *   [2] register address
 *   [3..6] 32-bit data, MSB first
 *   [7] CRC8
 */

#include "tmc2209.h"
#include <string.h>

#define TMC2209_UART_TIMEOUT_MS 20

/* Standard TMC CRC8 (datasheet section 5, "Calculating the CRC"). */
static uint8_t tmc2209_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t current_byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

void TMC2209_Init(TMC2209_t *drv, UART_HandleTypeDef *huart, uint8_t address)
{
    drv->huart = huart;
    drv->address = address;
    drv->last_ifcnt = 0;
}

bool TMC2209_WriteRegister(TMC2209_t *drv, uint8_t reg_addr, uint32_t data)
{
    uint8_t frame[8];
    frame[0] = 0x05;
    frame[1] = drv->address;
    frame[2] = reg_addr | 0x80;
    frame[3] = (uint8_t)(data >> 24);
    frame[4] = (uint8_t)(data >> 16);
    frame[5] = (uint8_t)(data >> 8);
    frame[6] = (uint8_t)(data);
    frame[7] = tmc2209_crc8(frame, 7);

    /* Half-duplex UART: our own TX will loop back on RX. Read it back and
     * discard so it doesn't get mistaken for a reply on the next call. */
    if (HAL_UART_Transmit(drv->huart, frame, sizeof(frame), TMC2209_UART_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    uint8_t echo[8];
    HAL_UART_Receive(drv->huart, echo, sizeof(echo), TMC2209_UART_TIMEOUT_MS);

    /* Verify the write landed by checking IFCNT incremented (skip this
     * check when writing IFCNT-adjacent bootstrapping regs to avoid
     * infinite recursion). */
    if (reg_addr != TMC2209_REG_IFCNT) {
        uint32_t ifcnt = 0;
        if (!TMC2209_ReadRegister(drv, TMC2209_REG_IFCNT, &ifcnt)) {
            return false; // couldn't verify, but write was sent
        }
        bool ok = (ifcnt != drv->last_ifcnt) || (drv->last_ifcnt == 0 && ifcnt == 0);
        drv->last_ifcnt = ifcnt;
        return ok;
    }
    return true;
}

bool TMC2209_ReadRegister(TMC2209_t *drv, uint8_t reg_addr, uint32_t *data_out)
{
    uint8_t req[4];
    req[0] = 0x05;
    req[1] = drv->address;
    req[2] = reg_addr; // write bit = 0
    req[3] = tmc2209_crc8(req, 3);

    if (HAL_UART_Transmit(drv->huart, req, sizeof(req), TMC2209_UART_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    /* Discard the echo of our own request (4 bytes on the shared line). */
    uint8_t echo[4];
    HAL_UART_Receive(drv->huart, echo, sizeof(echo), TMC2209_UART_TIMEOUT_MS);

    uint8_t reply[8];
    if (HAL_UART_Receive(drv->huart, reply, sizeof(reply), TMC2209_UART_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    uint8_t crc = tmc2209_crc8(reply, 7);
    if (crc != reply[7] || reply[0] != 0x05) {
        return false; // corrupted reply
    }

    *data_out = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
                ((uint32_t)reply[5] << 8) | (uint32_t)reply[6];
    return true;
}

bool TMC2209_Configure(TMC2209_t *drv, uint16_t microsteps)
{
    /* Clear GSTAT latched faults (write 1s to clear). */
    if (!TMC2209_WriteRegister(drv, TMC2209_REG_GSTAT, 0x07)) return false;

    /* GCONF: bit0 I_scale_analog=0 (use internal Vref via UART current),
     * bit7 pdn_disable=1 (required for UART mode - frees the PDN/UART pin),
     * bit6 mstep_reg_select=1 (microstep resolution set via CHOPCONF). */
    uint32_t gconf = (1 << 7) | (1 << 6);
    if (!TMC2209_WriteRegister(drv, TMC2209_REG_GCONF, gconf)) return false;

    /* CHOPCONF: map microsteps -> MRES field (0=256 ... 8=1). Common values
     * you'll actually use: 256->0, 16->4, 8->5. */
    uint8_t mres;
    switch (microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        default:  mres = 4; break; // fall back to 16 microsteps
    }
    uint32_t chopconf = (1UL << 0)              // TOFF: enable driver, ~fast decay time
                       | (3UL << 4)              // HSTRT
                       | (5UL << 7)              // TBL (blank time)
                       | ((uint32_t)mres << 24);  // MRES
    if (!TMC2209_WriteRegister(drv, TMC2209_REG_CHOPCONF, chopconf)) return false;

    /* TPOWERDOWN: delay before standstill current reduction kicks in. */
    if (!TMC2209_WriteRegister(drv, TMC2209_REG_TPOWERDOWN, 20)) return false;

    return true;
}

bool TMC2209_SetRunCurrent(TMC2209_t *drv, uint8_t run_current_0_31, uint8_t hold_current_0_31)
{
    uint32_t ihold_irun = ((uint32_t)hold_current_0_31 & 0x1F)
                         | (((uint32_t)run_current_0_31 & 0x1F) << 8)
                         | (5UL << 16); // IHOLDDELAY: ramp-down steps to hold current
    return TMC2209_WriteRegister(drv, TMC2209_REG_IHOLD_IRUN, ihold_irun);
}

bool TMC2209_GetStatus(TMC2209_t *drv, uint32_t *drv_status_out)
{
    return TMC2209_ReadRegister(drv, TMC2209_REG_DRV_STATUS, drv_status_out);
}

bool TMC2209_IsConnected(TMC2209_t *drv)
{
    uint32_t ioin = 0;
    if (!TMC2209_ReadRegister(drv, TMC2209_REG_IOIN, &ioin)) {
        return false;
    }
    /* IOIN[31:24] is the version byte and should read 0x21 for TMC2209. */
    return ((ioin >> 24) & 0xFF) == 0x21;
}