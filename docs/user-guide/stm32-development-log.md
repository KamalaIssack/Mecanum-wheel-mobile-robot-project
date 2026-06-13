# STM32 Development Log — Mecanum Wheel Mobile Robot

## Development Environment

- **MCU Board:** STM32 NUCLEO-F446RE (Cortex-M4, 180MHz, 512KB Flash, 128KB RAM)
- **IDE:** STM32CubeIDE 2.0 + STM32CubeMX 6.16.1 (separate applications)
- **Toolchain:** arm-none-eabi-gcc (via STM32CubeIDE)
- **Firmware Package:** STM32Cube FW_F4 V1.28.3
- **Host OS:** macOS
- **Debugger:** ST-LINK/V2-1 (on-board Nucleo)
- **Additional Software:** ST-LINK Server (required for flashing on macOS)

### STM32CubeIDE 2.0 Workflow Note

As of November 2025, ST released STM32CubeIDE 2.0 which **removed the integrated CubeMX** from the IDE. The new workflow requires two separate applications:

1. **STM32CubeMX** — Create projects, configure pins, peripherals, and clocks, then generate code
2. **STM32CubeIDE** — Import generated projects, write application code, build, debug, and flash

Project creation flow: CubeMX (Board Selector → Configure → Project Manager → Generate Code) → CubeIDE (File → Open Projects from File System → Import)

---

## Phase 1: LED Blink (GPIO Digital Output)

**Project:** `led_blink`
**Objective:** Verify the complete toolchain — create, build, flash, and run a program on the STM32

### Concept

The simplest embedded program: toggle a GPIO pin high and low with a delay. This tests every link in the chain from code editor to running hardware.

### CubeMX Configuration

- Board Selector: NUCLEO-F446RE
- All peripherals initialized with default mode (auto-configures LD2 on PA5 and User Button on PC13)
- Toolchain: STM32CubeIDE

### Code

In `main.c`, inside the `while(1)` loop:

```c
/* USER CODE BEGIN WHILE */
while (1)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
```

### Key Functions

| Function | Purpose |
|----------|---------|
| `HAL_GPIO_TogglePin(port, pin)` | Flips a GPIO pin between HIGH and LOW |
| `HAL_Delay(ms)` | Blocking delay in milliseconds |

### Result

- Build: **0 errors, 0 warnings** (5.3s build time)
- Memory: 8,612 bytes Flash (~1.7% of 512KB), 1,656 bytes RAM (~1.3% of 128KB)
- LED blinks on/off every 500ms ✓
- Verified faster blink at 100ms to confirm edit → build → flash cycle ✓

### Lessons Learned

- All user code must be placed between `/* USER CODE BEGIN */` and `/* USER CODE END */` comment pairs — CubeMX overwrites everything else during code regeneration
- ST-LINK Server must be installed separately on macOS for flashing to work
- ST-LINK firmware update may be required on first connection

---

## Phase 2: PWM LED Fade (Timer/PWM Output)

**Project:** `pwm_led_fade`
**Objective:** Learn hardware timer configuration and PWM generation — the same mechanism used for motor speed control

### Concept

Instead of toggling the LED fully on/off, PWM (Pulse Width Modulation) rapidly switches the pin at a fixed frequency while varying the duty cycle (ratio of on-time to off-time). At 1kHz, the switching is invisible to the eye, creating a smooth brightness control effect.

**This is directly applicable to motor control:** the BTS7960 motor drivers accept PWM signals where the duty cycle determines motor speed (0% = stopped, 100% = full speed).

### PWM Timer Math

The STM32F446RE's APB1 timer bus runs at **84 MHz**. To generate a 1kHz PWM signal with 1000 steps of resolution:

```
Timer Clock:    84,000,000 Hz (84 MHz)
Prescaler:      83          → 84,000,000 ÷ (83 + 1) = 1,000,000 Hz (1 MHz tick)
Counter Period: 999         → 1,000,000 ÷ (999 + 1) = 1,000 Hz (1 kHz PWM)
Resolution:     1000 steps  → duty cycle from 0 (off) to 999 (fully on)
```

**General formula:**

```
PWM Frequency = Timer Clock ÷ ((Prescaler + 1) × (Counter Period + 1))
```

This formula will be reused for motor PWM configuration.

> **Timer Bus Note:** On STM32F4, timer clocks on APB1 are doubled when the APB1 prescaler is not 1. Default CubeMX configuration results in 84 MHz timer clock for TIM2.

**PWM Frequency Selection for Motor Drivers:**

The BTS7960 drivers work well with PWM frequencies in the 1kHz–25kHz range. The choice involves a trade-off: 1kHz is audible but easy to debug with an oscilloscope or even by ear, while 20kHz is above human hearing and produces quieter motor operation. For initial development, 1kHz is preferred for easier troubleshooting.

### CubeMX Configuration

- Board Selector: NUCLEO-F446RE
- **TIM2 Channel 1** set to **PWM Generation CH1**
- PA5 remapped to TIM2_CH1 (default was PA0, but the on-board LED is on PA5)
- Parameter Settings:
  - Prescaler (PSC): **83**
  - Counter Period (ARR): **999**
  - Pulse (CH1): **0**
  - PWM Mode: PWM mode 1

### Code

In `main.c`, before the while loop (USER CODE BEGIN 2):

```c
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
/* USER CODE END 2 */
```

Inside the `while(1)` loop:

```c
while (1)
{
    // Fade in
    for (int i = 0; i <= 999; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
        HAL_Delay(1);
    }
    // Fade out
    for (int i = 999; i >= 0; i--)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
        HAL_Delay(1);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
```

### Key Functions

| Function | Purpose |
|----------|---------|
| `HAL_TIM_PWM_Start(&htim, channel)` | Starts PWM output on a timer channel |
| `__HAL_TIM_SET_COMPARE(&htim, channel, value)` | Sets the duty cycle (0 to Counter Period) |

### Result

- Build: 0 errors, 0 warnings ✓
- LED smoothly fades in and out with ~2 second cycle ✓
- Demonstrates PWM duty cycle control — same principle as motor speed control

### Mapping to Motor Control

| LED Fade | Motor Control |
|----------|---------------|
| PA5 (TIM2_CH1) | Motor PWM pins (TIM channels) |
| Duty cycle 0–999 | Motor speed 0–100% |
| 1kHz PWM frequency | 1kHz–25kHz for BTS7960 |
| `__HAL_TIM_SET_COMPARE` | Same function for motor speed |
| Single channel | 8 channels (2 per motor × 4 motors) |

---

## Repository Structure

```
firmware/stm32/nucleo_f446re/
    led_blink/                 ← Phase 1: GPIO basics
    pwm_led_fade/              ← Phase 2: Timer/PWM fundamentals
    single_motor_test/         ← Phase 3: One BTS7960 + one motor
    encoder_test/              ← Phase 4: Reading encoder feedback
    pid_motor_control/         ← Phase 5: PID velocity control
    uart_comm_test/            ← Phase 6: STM32 ↔ Raspberry Pi communication
    imu_test/                  ← Phase 7: MPU-9250 I2C integration
    mecanum_firmware/          ← Final: Complete robot firmware
```

---

## Phase 3: Single Motor Test (BTS7960 + PWM Direction Control)

**Project:** `single_motor_test`
**Objective:** Drive one TSINY-8370 DC motor through a BTS7960 motor driver using two PWM channels — one for forward, one for reverse — with variable speed control

### Concept

The BTS7960 is a half-bridge motor driver with two PWM inputs:

- **RPWM** — controls forward rotation (right/positive direction)
- **LPWM** — controls reverse rotation (left/negative direction)

To spin the motor, you activate one signal with a duty cycle and hold the other at 0. Speed maps linearly to duty cycle (0% = stopped, 100% = full speed). Both signals at 0 coasts the motor; both signals active simultaneously must be avoided (shoot-through).

This is the direct hardware bridge between the PWM theory from Phase 2 and the 4-motor mecanum drive system.

### BTS7960 Wiring

| BTS7960 Pin | STM32 Pin | Signal |
|-------------|-----------|--------|
| RPWM | PA6 (TIM3_CH1) | Forward PWM |
| LPWM | PA7 (TIM3_CH2) | Reverse PWM |
| R_EN | 3.3V (tied high) | Right bridge enable |
| L_EN | 3.3V (tied high) | Left bridge enable |
| VCC | 3.3V | Logic supply |
| GND | GND | Common ground |
| B+ | 12V supply | Motor power |
| B- | GND | Motor power return |
| M+ | Motor terminal A | — |
| M- | Motor terminal B | — |

> R_EN and L_EN are tied permanently high so the driver is always enabled. Direction and speed are controlled entirely through RPWM/LPWM duty cycles.

### CubeMX Configuration

- Board Selector: NUCLEO-F446RE
- **TIM3 Channel 1** → PWM Generation CH1 (PA6)
- **TIM3 Channel 2** → PWM Generation CH2 (PA7)
- Parameter Settings (same PWM math as Phase 2, same 1kHz target):
  - Prescaler (PSC): **83**
  - Counter Period (ARR): **999**
  - Pulse CH1: **0** (RPWM starts off)
  - Pulse CH2: **0** (LPWM starts off)
  - PWM Mode: PWM mode 1

### Code

Start both PWM channels before the loop (USER CODE BEGIN 2):

```c
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // RPWM
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // LPWM
/* USER CODE END 2 */
```

Direction and speed control functions (USER CODE BEGIN 0):

```c
/* USER CODE BEGIN 0 */
void motor_forward(uint16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);      // LPWM off
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);  // RPWM active
}

void motor_reverse(uint16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);      // RPWM off
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);  // LPWM active
}

void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
/* USER CODE END 0 */
```

Test sequence inside the `while(1)` loop:

```c
while (1)
{
    motor_forward(500);   // 50% speed forward
    HAL_Delay(2000);

    motor_stop();
    HAL_Delay(500);

    motor_reverse(500);   // 50% speed reverse
    HAL_Delay(2000);

    motor_stop();
    HAL_Delay(500);

    motor_forward(999);   // full speed forward
    HAL_Delay(2000);

    motor_stop();
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
```

### Key Functions

| Function | Purpose |
|----------|---------|
| `HAL_TIM_PWM_Start(&htim, channel)` | Starts PWM output on a timer channel |
| `__HAL_TIM_SET_COMPARE(&htim, channel, value)` | Sets duty cycle (0–999) |
| `motor_forward(speed)` | Drives RPWM, zeroes LPWM |
| `motor_reverse(speed)` | Drives LPWM, zeroes RPWM |
| `motor_stop()` | Zeroes both channels — coasts motor |

### Result

- Build: 0 errors, 0 warnings ✓
- Motor spins forward and reverse on command ✓
- Speed control via duty cycle confirmed ✓
- Direction switching with stop gap works cleanly ✓

![Phase 3 — Single Motor Test](../videos/stm32/phase3-motor-test.mp4)

### Mapping to Full Mecanum Drive

| Single Motor Test | Mecanum Firmware |
|-------------------|-----------------|
| TIM3 CH1/CH2 (1 motor) | 4 timers × 2 channels (4 motors) |
| `motor_forward/reverse` | Per-wheel direction functions |
| Fixed test sequence | Velocity commands from Raspberry Pi |
| 50% / 100% test speeds | PID-controlled target velocities |
