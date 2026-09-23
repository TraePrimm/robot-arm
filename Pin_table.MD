| Item | (info on what to put) |
|---|---|
| SPI1_SCK | PA5 |
| SPI1_MISO | PA6 |
| SPI1_MOSI | PD7 |
| Ethernet RMII pins (all) | (list whatever CubeMX assigned when LwIP was set up — copy from Pinout view, mark as claimed) |
| HAL timebase timer | (check `stm32h7xx_hal_timebase_tim.c` — note which TIMx FreeRTOS already owns, so it's excluded below) |
| Joint 0 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 0 — STEP | (free GPIO pin, Output mode) |
| Joint 0 — DIR | (free GPIO pin, Output mode) |
| Joint 0 — Step timer | (a TIMx not used elsewhere — e.g. htim2) |
| Joint 0 — TMC2209 UART bus + address | (e.g. huart1, address 0) |
| Joint 1 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 1 — STEP | (free GPIO pin, Output mode) |
| Joint 1 — DIR | (free GPIO pin, Output mode) |
| Joint 1 — Step timer | (a TIMx not used elsewhere — e.g. htim3) |
| Joint 1 — TMC2209 UART bus + address | (e.g. huart1, address 1) |
| Joint 2 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 2 — STEP | (free GPIO pin, Output mode) |
| Joint 2 — DIR | (free GPIO pin, Output mode) |
| Joint 2 — Step timer | (a TIMx not used elsewhere) |
| Joint 2 — TMC2209 UART bus + address | (e.g. huart1, address 2) |
| Joint 3 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 3 — STEP | (free GPIO pin, Output mode) |
| Joint 3 — DIR | (free GPIO pin, Output mode) |
| Joint 3 — Step timer | (a TIMx not used elsewhere) |
| Joint 3 — TMC2209 UART bus + address | (e.g. huart1, address 3) |
| Joint 4 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 4 — STEP | (free GPIO pin, Output mode) |
| Joint 4 — DIR | (free GPIO pin, Output mode) |
| Joint 4 — Step timer | (a TIMx not used elsewhere) |
| Joint 4 — TMC2209 UART bus + address | (e.g. huart2, address 0) |
| Joint 5 — Encoder CS | (free GPIO pin, Output mode, default level High) |
| Joint 5 — STEP | (free GPIO pin, Output mode) |
| Joint 5 — DIR | (free GPIO pin, Output mode) |
| Joint 5 — Step timer | (a TIMx not used elsewhere) |
| Joint 5 — TMC2209 UART bus + address | (e.g. huart2, address 1) |
| UART1 TX/RX pins | (the actual pins CubeMX assigns once you enable UART1 — record after) |
| UART2 TX/RX pins | (the actual pins CubeMX assigns once you enable UART2 — record after) |