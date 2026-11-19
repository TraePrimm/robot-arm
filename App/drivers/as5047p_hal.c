
/************************************************************************************
*  as5047p_hal.c - NUCLEO-H723ZG port for the CircularBuffer AS5047P driver
*                  (robot-arm firmware version)
*
*  The CS pin / SPI bus for each encoder id is NOT hard-coded here. encoder.c
*  registers them at init from the joint config (robot_arm_config.c), so the
*  pin table lives in one place only.
*
*  SPI (CubeMX): mode 1 (CPOL low, CPHA 2 edge), 16-bit, MSB first, <= 10 MHz,
*                Master Keep IO State enabled. CS pins: GPIO_Output, level High.
*
*  Fixes vs. the upstream template:
*   - Template returned -1 from uint8_t functions and tested "< 0" (never true),
*     so SPI errors were swallowed. Every failure here returns 1.
*   - 350 ns CS timing gaps enforced with the DWT cycle counter.
*
*  Not thread-safe: call the driver from ONE task only (ControlLoop).
************************************************************************************/
 
#include "as5047p_hal.h"
#include "stm32h7xx_hal.h"
 
#define AS_MAX_DEV 6
 
typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef      *port;
  uint16_t           pin;
} as_dev_t;
 
static as_dev_t s_dev[AS_MAX_DEV];
 
/*--- Called by encoder.c before AS5047P_Init(id) ---------------------------------*/
uint8_t AS5047P_HAL_Register(uint16_t id, SPI_HandleTypeDef *spi,
                             GPIO_TypeDef *port, uint16_t pin)
{
  if (id >= AS_MAX_DEV || spi == NULL || port == NULL) return 1;
  s_dev[id].spi  = spi;
  s_dev[id].port = port;
  s_dev[id].pin  = pin;
  return 0;
}
 
/*--- Nanosecond delay on the DWT cycle counter -----------------------------------*/
static void dwt_init(void)
{
  static uint8_t done = 0;
  if (done) return;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;              /* unlock DWT on Cortex-M7 */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  done = 1;
}
 
static void delay_ns(uint32_t ns)
{
  uint32_t cycles = (uint32_t)(((uint64_t)SystemCoreClock * ns) / 1000000000ULL) + 1;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) { }
}
 
void AS5047P_HAL_Delay_ms(uint32_t ms)
{
  HAL_Delay(ms);   /* only used at init / zeroing, never in the 1 kHz loop */
}
 
void AS5047P_HAL_Debug(uint16_t id, int16_t errorCode, char *errMsg)
{
  (void)id; (void)errorCode; (void)errMsg;
}
 
uint8_t AS5047P_HAL_Init(uint16_t id)
{
  if (id >= AS_MAX_DEV || s_dev[id].spi == NULL) return 1;   /* not registered */
  dwt_init();
  HAL_GPIO_WritePin(s_dev[id].port, s_dev[id].pin, GPIO_PIN_SET);
  delay_ns(500);
  return 0;
}
 
uint8_t AS5047P_HAL_SPI_Transaction(uint16_t *pTx, uint16_t *pRx, uint16_t id)
{
  if (id >= AS_MAX_DEV || s_dev[id].spi == NULL) return 1;
  const as_dev_t *d = &s_dev[id];
 
  HAL_GPIO_WritePin(d->port, d->pin, GPIO_PIN_RESET);
  delay_ns(400);   /* CS falling -> first SCK edge >= 350 ns */
 
  /* Size = 1: SPI is in 16-bit mode, so one frame = one 16-bit word. */
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(d->spi, (uint8_t *)pTx,
                                                 (uint8_t *)pRx, 1, 2);
 
  delay_ns(600);   /* last SCK edge -> CS rising >= 1/2 SCK period at >= 1 MHz */
  HAL_GPIO_WritePin(d->port, d->pin, GPIO_PIN_SET);
  delay_ns(400);   /* CS high time between frames >= 350 ns */
 
  return (st == HAL_OK) ? 0 : 1;
}
 