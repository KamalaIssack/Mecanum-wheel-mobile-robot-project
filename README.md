# Mecanum Wheel Mobile Robot Project

A mobile robot platform utilizing mecanum wheels for omnidirectional movement, built around the STM32 NUCLEO-F446RE microcontroller.

## Project Overview

This project implements a mecanum wheel mobile robot capable of:
- Omnidirectional movement (forward, backward, lateral, diagonal)
- Rotation in place
- Simultaneous translation and rotation

The firmware is being developed incrementally through a structured phase-based approach, starting from basic GPIO control and building up to a complete mecanum drive system with sensor feedback and PID control.

## Current Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | LED Blink - GPIO digital output | Completed |
| 2 | PWM LED Fade - Timer/PWM control | Completed |
| 3 | Single Motor Test - BTS7960 driver control | Planned |
| 4 | Encoder Test - Motor encoder feedback | Planned |
| 5 | PID Motor Control - Closed-loop velocity control | Planned |
| 6 | UART Communication - STM32 to Raspberry Pi | Planned |
| 7 | IMU Integration - MPU-9250 via I2C | Planned |
| 8 | Complete Mecanum Firmware - Full system integration | Planned |

## Project Structure

```
.
├── src/                    # Source code
│   ├── control/            # Motion control algorithms
│   ├── kinematics/         # Mecanum wheel kinematics
│   ├── sensors/            # Sensor integration
│   ├── communication/      # Communication protocols
│   └── utils/              # Utility functions
├── docs/                   # Documentation
│   ├── api/                # API documentation
│   ├── user-guide/         # User guides and development log
│   └── images/             # Documentation images
├── hardware/               # Hardware documentation
│   ├── cad/                # CAD files and 3D models
│   ├── schematics/         # Electrical schematics
│   ├── bom/                # Bill of materials
│   └── datasheets/         # Component datasheets
├── firmware/               # Microcontroller firmware
│   ├── stm32/              # STM32 firmware (active development)
│   │   └── nucleo_f446re/  # NUCLEO-F446RE board projects
│   │       ├── led_blink/          # Phase 1: GPIO output
│   │       └── pwm_led_fade/       # Phase 2: PWM control
│   ├── arduino/            # Arduino-based firmware
│   ├── esp32/              # ESP32-based firmware
│   └── drivers/            # Motor and sensor drivers
├── config/                 # Configuration files
├── tests/                  # Test files
└── scripts/                # Utility scripts
```

## Hardware Components

- **Microcontroller**: STM32 NUCLEO-F446RE (ARM Cortex-M4, 180 MHz, 512 KB Flash, 128 KB RAM)
- **Motor Drivers**: BTS7960 (PWM-based, 1-25 kHz frequency range)
- **Motors**: 4x DC motors with encoders (TSINY-8370)
- **Wheels**: 4x Mecanum wheels
- **IMU**: MPU-9250 (I2C interface) - planned
- **Power Supply/Battery**: TBD
- **Chassis/Frame**: TBD

## Getting Started

### Prerequisites

- [STM32CubeIDE 2.0](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeMX 6.16.1](https://www.st.com/en/development-tools/stm32cubemx.html) (separate application for code generation)
- ST-LINK Server (required on macOS for flashing)
- STM32Cube FW_F4 V1.28.3 firmware package
- USB cable for ST-LINK/V2-1 on-board debugger

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Mecanum-wheel-mobile-robot-project.git
   ```

2. Open STM32CubeIDE and import the desired firmware project from `firmware/stm32/nucleo_f446re/`

3. Use STM32CubeMX to generate/regenerate peripheral configuration code as needed

4. Build and flash to the NUCLEO-F446RE board via the on-board ST-LINK debugger

### Development Workflow

1. Configure peripherals in STM32CubeMX (`.ioc` file)
2. Generate code (placed within `/* USER CODE BEGIN/END */` blocks to survive regeneration)
3. Build in STM32CubeIDE
4. Flash via ST-LINK

## Mecanum Wheel Kinematics

The mecanum wheel configuration allows for omnidirectional movement through independent control of each wheel's velocity.

| Movement      | FL  | FR  | BL  | BR  |
|---------------|-----|-----|-----|-----|
| Forward       | +   | +   | +   | +   |
| Backward      | -   | -   | -   | -   |
| Strafe Left   | -   | +   | +   | -   |
| Strafe Right  | +   | -   | -   | +   |
| Rotate CW     | +   | -   | +   | -   |
| Rotate CCW    | -   | +   | -   | +   |

### PWM Motor Control

Each motor is controlled via a BTS7960 driver using two PWM channels:
- **RPWM**: Forward direction
- **LPWM**: Reverse direction
- **Duty cycle**: 0-999 steps (1000 resolution levels)
- **PWM frequency**: 1 kHz for development, 20 kHz for production (reduces audible noise)
- **Timer source**: APB1 at 84 MHz, prescaler 83 for 1 MHz tick, period 999 for 1 kHz output

## Technical Notes

- The STM32F446RE currently uses ~1.7% Flash and ~1.3% RAM, leaving plenty of room for the full firmware
- The system will support up to 8 PWM channels (2 per motor x 4 motors)
- User code must be placed within `/* USER CODE BEGIN/END */` blocks to survive STM32CubeMX code regeneration

For detailed development notes and phase documentation, see [`docs/user-guide/stm32-development-log.md`](docs/user-guide/stm32-development-log.md).

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Open a Pull Request

## License

[Add license information]

## Acknowledgments

[Add acknowledgments]
