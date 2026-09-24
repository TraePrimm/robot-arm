| Item | (info on what to put) |
|---|---|
| SPI1_SCK | PA5 |
| SPI1_MISO | PA6 |
| SPI1_MOSI | PD7 |
| Ethernet RMII pins (all) | Ethernet RMII pins (all)
ETH_MDC: PC1; ETH_REF_CLK: PA1; ETH_MDIO: PA2; ETH_TXD1: PB13; ETH_TXD0: PB12; ETH_TX_EN: PB11; ETH_CRS_DV: PA7; ETH_RXD0: PC4; ETH_RXD1: PC5 |
| HAL timebase timer | TIM1 |
| Joint 0 — Encoder CS | PA4 (Output, High default) |
| Joint 0 — STEP | PA0 (Output, via TIM2_CH1) |
| Joint 0 — DIR | 	PC0 (Output) |
| Joint 0 — Step timer | TIM2 (CH1) — ⚠ Prescaler/ARR not yet configured|
| Joint 0 — TMC2209 UART bus + address | huart1, address 0 |
| Joint 1 — Encoder CS | PB2 (Output, High default) |
| Joint 1 — STEP | PC8 (via TIM3_CH3) |
| Joint 1 — DIR | PC9 (Output) |
| Joint 1 — Step timer | TIM3 (CH3) — ⚠ Prescaler/ARR not yet configured |
| Joint 1 — TMC2209 UART bus + address | huart1, address 1 |
| Joint 2 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 2 — STEP | (free GPIO pin, Output mode) |
| Joint 2 — DIR | (free GPIO pin, Output mode) |
| Joint 2 — Step timer | (a TIMx not used elsewhere) |
| Joint 2 — TMC2209 UART bus + address | huart1, address 2 |
| Joint 3 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 3 — STEP | (free GPIO pin, Output mode) |
| Joint 3 — DIR | (free GPIO pin, Output mode) |
| Joint 3 — Step timer | (a TIMx not used elsewhere) |
| Joint 3 — TMC2209 UART bus + address | huart1, address 3 |
| Joint 4 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 4 — STEP | (free GPIO pin, Output mode) |
| Joint 4 — DIR | (free GPIO pin, Output mode) |
| Joint 4 — Step timer | (a TIMx not used elsewhere) |
| Joint 4 — TMC2209 UART bus + address | huart2, address 0|
| Joint 5 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 5 — STEP | (free GPIO pin, Output mode) |
| Joint 5 — DIR | (free GPIO pin, Output mode) |
| Joint 5 — Step timer | (a TIMx not used elsewhere) |
| Joint 5 — TMC2209 UART bus + address | huart2, address 1 |
| UART1 TX/RX pins | PA9 (single wire — one pin serves both) |
| UART2 TX/RX pins | (the actual pins CubeMX assigns once you enable UART2 — record after) |