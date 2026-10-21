/* tmc2209_cfg.h: Layer D. Puts one TMC2209 into a known configuration. */
#ifndef TMC2209_CFG_H
#define TMC2209_CFG_H

#include "tmc2209_reg.h"

typedef struct {
    uint16_t microsteps;     /* 1,2,4,...,256 (power of two). You want 16. */
    uint8_t  run_current;    /* IRUN,  0-31 (fraction of full-scale current) */
    uint8_t  hold_current;   /* IHOLD, 0-31 */
    uint8_t  hold_delay;     /* IHOLDDELAY, 0-15: ramp-down steps run -> hold */
    uint8_t  tpowerdown;     /* TPOWERDOWN, 0-255: wait before dropping to hold */
    uint8_t  senddelay;      /* SLAVECONF SENDDELAY, 0-15. Use >= 2 with several drivers on one bus */
    bool     interpolate;    /* INTPOL: chip interpolates microsteps up to 256 */
} tmc_cfg_t;

/* Safe starting values: 16 microsteps, LOW currents, interpolation on. */
tmc_cfg_t tmc_cfg_default(void);

/* Reads IOIN and checks the version byte is 0x21. */
tmc_status_t tmc_check_version(tmc_dev_t *d);

/* Full bring-up: version check, clear GSTAT, SLAVECONF, GCONF, CHOPCONF,
 * IHOLD_IRUN, TPOWERDOWN. Every write is confirmed via IFCNT. */
tmc_status_t tmc_configure(tmc_dev_t *d, const tmc_cfg_t *cfg);

#endif /* TMC2209_CFG_H */