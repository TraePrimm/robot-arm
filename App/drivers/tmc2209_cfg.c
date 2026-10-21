/* tmc2209_cfg.c: Layer D. See tmc2209_cfg.h.
 *
 * Bit positions below are from memory and cross-checked against your notes.
 * VERIFY each one against the datasheet register table before trusting it:
 *   GCONF:      pdn_disable = bit 6, mstep_reg_select = bit 7
 *   CHOPCONF:   MRES = bits 27:24, INTPOL = bit 28
 *   IHOLD_IRUN: IHOLD = 4:0, IRUN = 12:8, IHOLDDELAY = 19:16
 *   SLAVECONF:  SENDDELAY = bits 11:8
 *   IOIN:       version byte = bits 31:24 (0x21)
 */
#include "tmc2209_cfg.h"

#define GCONF_PDN_DISABLE        (1UL << 6)   /* required for UART use */
#define GCONF_MSTEP_REG_SELECT   (1UL << 7)   /* microsteps come from CHOPCONF.MRES */

#define CHOPCONF_MRES_SHIFT      24
#define CHOPCONF_MRES_MASK       (0xFUL << CHOPCONF_MRES_SHIFT)
#define CHOPCONF_INTPOL          (1UL << 28)

#define TMC_VERSION_TMC2209      0x21

tmc_cfg_t tmc_cfg_default(void)
{
    tmc_cfg_t c = {
        .microsteps   = 16,
        .run_current  = 8,     /* low on purpose; raise once the motor is wired */
        .hold_current = 4,
        .hold_delay   = 5,
        .tpowerdown   = 20,
        .senddelay    = 2,
        .interpolate  = true,
    };
    return c;
}

/* Read-modify-write: change only the bits in 'mask', keep everything else the
 * chip currently has (its reset defaults for the fields we don't touch). */
static tmc_status_t rmw(tmc_dev_t *d, uint8_t reg, uint32_t mask, uint32_t bits)
{
    uint32_t v;
    tmc_status_t st = tmc_read_reg(d, reg, &v);
    if (st != TMC_OK) return st;
    return tmc_write_reg(d, reg, (v & ~mask) | (bits & mask));
}

/* microsteps -> MRES field: 256 -> 0, 128 -> 1, ... 1 -> 8. Returns -1 if invalid. */
static int mres_from_microsteps(uint16_t ms)
{
    if (ms < 1 || ms > 256 || (ms & (ms - 1)) != 0) return -1;
    int mres = 0;
    for (uint16_t v = 256; v > ms; v >>= 1) mres++;
    return mres;
}

tmc_status_t tmc_check_version(tmc_dev_t *d)
{
    uint32_t ioin;
    tmc_status_t st = tmc_read_reg(d, TMC_REG_IOIN, &ioin);
    if (st != TMC_OK) return st;
    return ((ioin >> 24) & 0xFF) == TMC_VERSION_TMC2209 ? TMC_OK : TMC_ERR_WRONG_CHIP;
}

tmc_status_t tmc_configure(tmc_dev_t *d, const tmc_cfg_t *cfg)
{
    int mres = mres_from_microsteps(cfg->microsteps);
    if (mres < 0 || cfg->run_current > 31 || cfg->hold_current > 31 ||
        cfg->hold_delay > 15 || cfg->senddelay > 15)
        return TMC_ERR_BAD_ARG;

    tmc_status_t st;

    if ((st = tmc_check_version(d)) != TMC_OK) return st;

    /* Clear latched flags (reset / drv_err / uv_cp): write 1s to clear. */
    if ((st = tmc_write_reg(d, TMC_REG_GSTAT, 0x07)) != TMC_OK) return st;

    /* Reply delay. Do this early; >= 2 is needed with several drivers per bus. */
    if ((st = tmc_write_reg(d, TMC_REG_SLAVECONF, (uint32_t)cfg->senddelay << 8)) != TMC_OK) return st;

    /* GCONF: set the two bits UART mode needs, leave all others untouched.
     * NOTE: I_scale_analog (bit 0) is left at its default, so the VREF pin
     * on your breakout still sets the maximum motor current. */
    st = rmw(d, TMC_REG_GCONF, GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT,
                               GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT);
    if (st != TMC_OK) return st;

    /* CHOPCONF: only MRES and INTPOL; TOFF/HSTRT/HEND/TBL keep chip defaults. */
    uint32_t bits = ((uint32_t)mres << CHOPCONF_MRES_SHIFT)
                  | (cfg->interpolate ? CHOPCONF_INTPOL : 0);
    st = rmw(d, TMC_REG_CHOPCONF, CHOPCONF_MRES_MASK | CHOPCONF_INTPOL, bits);
    if (st != TMC_OK) return st;

    uint32_t ihold_irun = ((uint32_t)cfg->hold_current)
                        | ((uint32_t)cfg->run_current << 8)
                        | ((uint32_t)cfg->hold_delay  << 16);
    if ((st = tmc_write_reg(d, TMC_REG_IHOLD_IRUN, ihold_irun)) != TMC_OK) return st;

    if ((st = tmc_write_reg(d, TMC_REG_TPOWERDOWN, cfg->tpowerdown)) != TMC_OK) return st;

    return TMC_OK;
}