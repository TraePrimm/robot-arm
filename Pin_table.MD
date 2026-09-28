| Item | (info on what to put) |
|---|---|
| SPI1_SCK | PG11 |
| SPI1_MISO | PA6 |
| SPI1_MOSI | PD7 |
| Ethernet RMII pins (all) | Ethernet RMII pins (all)
ETH_MDC: PC1; ETH_REF_CLK: PA1; ETH_MDIO: PA2; ETH_TXD1: PB13; ETH_TXD0: PB12; ETH_TX_EN: PB11; ETH_CRS_DV: PA7; ETH_RXD0: PC4; ETH_RXD1: PC5 |
| HAL timebase timer | TIM1 |
| Joint 0 — Encoder CS | PA4 (Output, High default) |
| Joint 0 — STEP | PA5 (Output, via TIM2_CH1) |
| Joint 0 — DIR | 	PC0 (Output) |
| Joint 0 — Step timer | TIM2 (CH1) — ⚠ Prescaler/ARR not yet configured|
| Joint 0 — TMC2209 UART bus + address | huart1, address 0 |
| Joint 1 — Encoder CS | PB2 (Output, High default) |
| Joint 1 — STEP | PC8 (via TIM3_CH3) |
| Joint 1 — DIR | PC9 (Output) |
| Joint 1 — Step timer | TIM3 (CH3) — ⚠ Prescaler/ARR not yet configured |
| Joint 1 — TMC2209 UART bus + address | huart1, address 1 |
| Joint 2 — Encoder CS | PC2 (Output, High default)|
| Joint 2 — STEP | PD12 (via TIM4_CH1) |
| Joint 2 — DIR | PC3 (Output) |
| Joint 2 — Step timer | TIM4 (CH1) |
| Joint 2 — TMC2209 UART bus + address | huart1, address 2 |
| Joint 3 — Encoder CS |PC10 (Output, High default)|
| Joint 3 — STEP | PC6 (via TIM8_CH1) |
| Joint 3 — DIR | PC11 (Output) |
| Joint 3 — Step timer | TIM8 (CH1)|
| Joint 3 — TMC2209 UART bus + address | huart1, address 3 |
| Joint 4 — Encoder CS | PD0 (Output, High default)|
| Joint 4 — STEP | PB15 (via TIM12_CH2) |
| Joint 4 — DIR | PD1 (Output) |
| Joint 4 — Step timer | 	TIM12 (CH2) |
| Joint 4 — TMC2209 UART bus + address | huart2, address 0|
| Joint 5 — Encoder CS | PD2 (Output, High default) |
| Joint 5 — STEP | PA0 (via TIM5_CH1) |
| Joint 5 — DIR | PD3 (Output) |
| Joint 5 — Step timer | TIM5 (CH1) |
| Joint 5 — TMC2209 UART bus + address | huart2, address 1 |
| UART1 TX/RX pins | PA9 (single wire — one pin serves both) |
| UART2 TX/RX pins | PD5 (single wire — one pin serves both)|