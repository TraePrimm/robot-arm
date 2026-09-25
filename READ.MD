This is my custom robot arm. 

TO build it run: make -j$(nproc)

In terms of where I am at right now it's setup up pins so fun...







# Stepper Timer Math (STEP frequency via Prescaler/ARR)

**Timer input clock:** 275 MHz (both APB1 and APB2 timer clocks on this build)

## Step 1 — Prescaler → tick rate
```
Tick_Rate (Hz) = Timer_Clock / (Prescaler + 1)
```
Using Prescaler = 274 → Tick_Rate = 275,000,000 / 275 = 1,000,000 Hz (1 MHz), same for all 6 joint timers.

## Step 2 — Steps per output revolution
```
Steps_Per_Output_Rev = Motor_Full_Steps_Per_Rev × Microstepping × Gear_Ratio
```
- Motor_Full_Steps_Per_Rev = 200 (1.8°/step NEMA17)
- Microstepping = 16 (1/16, via TMC2209 UART config)
- Gear_Ratio = joint-specific (Joint 0 = 36:1, Joints 1–5 = 12:1)

## Step 3 — Required STEP pulse frequency for a target output RPM
```
Step_Freq (Hz) = (Steps_Per_Output_Rev × Target_RPM) / 60
```

## Step 4 — ARR (Counter Period) from tick rate and step frequency
```
ARR = (Tick_Rate / Step_Freq) − 1
```

## Step 5 — Back-check actual achieved RPM (ARR is an integer, so this won't be exact)
```
Actual_Step_Freq = Tick_Rate / (ARR + 1)
Actual_RPM = (Actual_Step_Freq × 60) / Steps_Per_Output_Rev
```

## To change speed later
Keep Prescaler fixed at 274 (1 MHz tick rate). Solve Step 3 → Step 4 for the new Target_RPM, update only ARR (Counter Period) in CubeMX, regenerate.

---

## Current values (1/16 microstepping, 20 RPM target)

| Joint | Gear Ratio | Steps/Output-Rev | Step Freq (Hz) | Prescaler | ARR |
|---|---|---|---|---|---|
| 0 | 36:1 | 115,200 | 38,400 | 274 | 25 |
| 1–5 | 12:1 | 38,400 | 12,800 | 274 | 77 |
