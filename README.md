# Mecanum Wheel Mobile Robot Project

**A four-wheel mecanum-drive robot that drives under its own battery power today, with all four encoders reporting live closed-loop-ready velocity feedback.**

Most "autonomous robot" portfolio projects stop at Gazebo. This one doesn't: the physical robot has driven under battery power with wheels off the ground, all four BTS7960-driven motors commanded through a real mecanum mixer, and all four quadrature encoders confirmed reading live on hardware. The ROS 2 stack — inverse/forward kinematics, dead-reckoning odometry, and a full Gazebo Harmonic simulation validated to 9 significant figures against the robot's own math — is built and tested in parallel, targeting Nav2 autonomous navigation once the STM32↔Raspberry Pi serial link, sensor fusion, and SLAM land.

The system splits across two boards by responsibility:

- **STM32 Nucleo-F446RE** — the real-time layer. Deterministic PWM generation, quadrature encoder reading, and (soon) closed-loop velocity control, with no OS in the way.
- **Raspberry Pi 5** — the perception and autonomy layer, running ROS 2 Jazzy: kinematics, odometry, sensor fusion, SLAM, and Nav2.

The two boards will talk over a serial link (USART2 on the Nucleo side) — the protocol is in development now, ahead of the ROS 2-side bridge node. See [System Architecture](#system-architecture) for the full breakdown.

## Project Overview

This project implements a mecanum wheel mobile robot capable of:
- Omnidirectional movement (forward, backward, lateral, diagonal)
- Rotation in place
- Simultaneous translation and rotation
- LiDAR-based mapping and autonomous navigation (planned)

The build follows a two-deck chassis: Deck 1 carries the power distribution layer (complete), and Deck 2 will carry the compute and sensing layer (in progress). Firmware and ROS 2 software are developed incrementally through a structured phase-based approach.

---

## Demo

**First powered drive** — all four wheels driven forward under battery power, wheels off the ground, all four encoders confirmed reading live on the debugger.

https://github.com/user-attachments/assets/70353340-a9e4-4cdd-b505-c8343d9dc1bf

**Encoder velocities on the debugger** — live per-wheel velocity readouts via STM32 Live Expressions, confirming the forward-positive sign convention on hardware.

https://github.com/user-attachments/assets/f964e577-ef93-403b-8885-005ee88034cf

---

## Current Status

### Hardware Build

| Stage | Description | Status |
|-------|-------------|--------|
| H1 | Chassis frame assembly | ✅ Complete |
| H2 | Motor and wheel mounting | ✅ Complete |
| H3 | Deck 1 – Power wiring (bus bars, fuse, BTS7960, bucks) | ✅ Complete |
| H4 | Control-side wiring (encoder signal wires + BTS7960 logic connections, continuity-checked) | ✅ Complete |
| H5 | Deck 2 – Compute and sensing layer mounting | 🔄 In Progress |
| H6 | Full system integration and cable management | ⏳ Planned |

### Firmware (STM32 Nucleo-F446RE)

| Stage | Description | Status |
|-------|-------------|--------|
| F1 | Bring-up: GPIO LED blink, PWM LED fade, single-motor BTS7960 test | ✅ Complete |
| F2 | Full 4-motor mecanum drive — `mecanum_drive()` / `motor_set()`, 8 PWM channels on TIM3/TIM4 @ 1 kHz | ✅ Complete |
| F3 | All four wheel encoders read via hardware quadrature timers (TIM1, TIM8, TIM5, TIM2); 50 Hz velocity computed via a TIM6 interrupt | ✅ Complete |
| F4 | Forward-positive per-wheel sign convention, verified on hardware | ✅ Complete |
| F5 | Disarm-at-boot safety layer (`motors_arm()` gate, duty-cycle ceiling), hardware-verified with the debugger | ✅ Complete |
| F6 | First powered drive — all four wheels under battery, all four encoders confirmed live | ✅ Complete |
| F7 | Serial protocol over USART2, tested against a Mac Python script | ⏳ Planned |
| F8 | Command watchdog | ⏳ Planned |
| F9 | Closed-loop PID velocity control | ⏳ Planned |

### ROS 2 (Raspberry Pi 5, `ros2_ws/`)

| Stage | Description | Status |
|-------|-------------|--------|
| R1 | Package scaffolding — `mecanum_bringup`, `mecanum_description`, `mecanum_interfaces`, `mecanum_control` | ✅ Complete |
| R2 | `mecanum_kinematics` node — inverse kinematics, `/cmd_vel` → `/wheel_speeds` | ✅ Complete |
| R3 | `mecanum_odometry` node — forward kinematics, dead-reckoning odometry, publishes `/odom` and the `odom`→`base_link` TF | ✅ Complete |
| R4 | Gazebo Harmonic simulation — headless, `ros_gz` bridge; kinematics/odometry validated against the simulator to 9 significant figures; visualized in Foxglove | ✅ Complete |
| R5 | Nucleo serial-bridge node | ⏳ Planned |
| R6 | IMU fusion via `robot_localization` | ⏳ Planned |
| R7 | RPLIDAR `/scan` integration | ⏳ Planned |
| R8 | SLAM (`slam_toolbox`) | ⏳ Planned |
| R9 | Nav2 autonomous navigation | ⏳ Planned |

---

## Safety Architecture

There is **no physical e-stop button** on this robot — that hardware was deliberately dropped in favor of a layered electrical/firmware approach instead:

1. **Physical kill — XT60 battery disconnect.** Unconditional, always available: pull the connector and every driver loses power.
2. **Automatic kill — firmware command watchdog** *(planned)*. If the serial link to the Raspberry Pi goes quiet, the Nucleo will disarm itself.
3. **Deliberate kill — explicit disarm.** The board boots disarmed: all four BTS7960 enable pins are held low at startup, and an explicit `motors_arm()` call is required before any PWM reaches the drivers. Disarming drops the enable pins **and** zeros the PWM duty registers. A duty-cycle ceiling (300 of 999) also caps speed during development. This layer is implemented and hardware-verified with the debugger today.

---

## Build Log

### Deck 1 – Power Wiring ✅ Complete

**Goal:** Wire all power components on Deck 1 following a star topology so every consumer gets a clean, independently fused feed from the central bus bars.

**What was done:**
- Mounted 4× BTS7960 motor drivers (one per wheel: FL, FR, RL, RR)
- Mounted 2× XL4016E1 buck converters (Buck A and Buck B)
- Installed positive and negative bus bars as the central power distribution point
- Wired XT60 battery connector → inline fuse → positive bus bar
- Star-wired all BTS7960 PWR inputs and both buck converter inputs from the bus bars
- Adjusted and verified output voltages with a multimeter (the multimeter reading is treated as ground truth, not the buck modules' onboard displays):
  - **Buck A → 5.00 V** (STM32 Nucleo supply)
  - **Buck B → 5.10 V** (Raspberry Pi 5 supply)

**Result:** Deck 1 power wiring complete and tested. All voltages verified. Ready for Deck 2 assembly.

| Isometric view | Top-down view |
|:-:|:-:|
| ![Deck 1 isometric: yellow platform with 2 mecanum wheels visible on the left, 4× BTS7960 drivers at the corners, 2× XL4016E1 bucks with LED displays in the centre, and positive bus bar above them. Red wires run from the bus bar to each driver and buck.](docs/images/hardware/deck1-power-wiring-iso.png) | ![Deck 1 top-down: all 4 mecanum wheels visible at the corners. Positive bus bar at top-centre, negative bus bar below it, 2× XL4016E1 bucks side-by-side in the middle, and 4× BTS7960 (IBT-2) drivers — 2 at the top and 2 at the bottom — each fed directly from the bus bars in a star topology.](docs/images/hardware/deck1-power-wiring-top.png) |

---

### Control-Side Wiring ✅ Complete

**Goal:** Get every signal wire between Deck 1's drive electronics and the Nucleo in place and verified, ahead of Deck 2 going on top.

**What was done:**
- Ran all four encoder signal wires from the motors to the Nucleo
- Wired all four BTS7960 logic connections — RPWM, LPWM, R_EN, L_EN per driver
- Tied a common logic ground across all four drivers and the Nucleo GND
- Continuity-checked every connection

**Result:** The Nucleo now has everything it needs, electrically, to drive all four motors and read all four encoders — confirmed by the first powered drive below.

---

### Deck 2 – Compute and Sensing Layer 🔄 In Progress

**Goal:** Mount and connect all compute and sensing components on Deck 2.

**Planned components:**
- Raspberry Pi 5 (compute)
- STM32 Nucleo-F446RE (motor firmware)
- RPLIDAR C1 (360° LiDAR)
- MPU-9250 IMU (inertial measurement)
- Pass-through grommets for encoder and driver logic cables from Deck 1

**Status:** Hole layout planned and drilling guide prepared. Mounting not yet started.

---

## Roadmap to Completion

Two tracks can proceed largely in parallel — firmware software work doesn't need Deck 2 mounted, since Deck 1 and the control-side wiring already give the Nucleo everything it needs.

| Stage | Description | Depends on |
|-------|-------------|------------|
| 1 | Serial protocol over USART2, validated against a Mac Python test script | Firmware track — in progress now |
| 2 | Command watchdog (automatic kill on lost link) | Stage 1 |
| 3 | Closed-loop PID velocity control | Stage 2 |
| 4 | Deck 2 mounting (Raspberry Pi 5, RPLIDAR C1, IMU) | Hardware track — in progress now |
| 5 | Full cable management and system integration | Stage 4 |
| 6 | ROS 2 Nucleo serial-bridge node | Stages 1 and 5 |
| 7 | IMU fusion via `robot_localization` | Stage 6 |
| 8 | RPLIDAR `/scan` integration + SLAM (`slam_toolbox`) | Stage 6 |
| 9 | Nav2 autonomous navigation | Stages 7 and 8 |

---

## Hardware Components

### Compute and Control

| Component | Part | Notes |
|-----------|------|-------|
| Main compute | Raspberry Pi 5 (8 GB) | Runs ROS 2, SLAM, Nav2 |
| Microcontroller | STM32 Nucleo-F446RE | ARM Cortex-M4, 180 MHz, 512 KB Flash |
| IMU | MPU-9250 | I2C interface |
| LiDAR | RPLIDAR C1 | 360° scanning, USB interface |

### Power System

| Component | Part | Notes |
|-----------|------|-------|
| Battery | 3S LiPo (~11.1 V nominal) | XT60 connector |
| Inline fuse | 30 A | Battery-side protection |
| Bus bars | Positive + negative | Star topology distribution point |
| Buck A | XL4016E1 | Set to 5.00 V → STM32 Nucleo |
| Buck B | XL4016E1 | Set to 5.10 V → Raspberry Pi 5 |

### Drive System

| Component | Part | Quantity |
|-----------|------|---------|
| Motor drivers | BTS7960 | 4× (one per wheel) |
| Motors | TSINY-8370 DC with encoder | 4× |
| Wheels | Mecanum wheels | 4× |

---

## Robot Geometry

```
        Front
   FL ──────── FR
    |           |
    |  lx   lx  |
    |           |
   RL ──────── RR
        Rear
```

| Parameter | Value |
|-----------|-------|
| Wheel-contact layout | 200 × 200 mm square |
| Wheel radius (r) | 0.0483 m (caliper-measured, confirmed) |
| Half-wheelbase (lx) | 0.1 m |
| Half-track (ly) | 0.1 m |
| Kinematic origin | Centre of the wheel rectangle; `base_link` sits at ground level at this centroid |

Wheel naming: **FL** (front-left), **FR** (front-right), **RL** (rear-left), **RR** (rear-right).

---

## Project Structure

```
.
├── firmware/               # Microcontroller firmware
│   ├── stm32/
│   │   └── nucleo_f446re/
│   │       ├── led_blink/        # Bring-up phase
│   │       ├── pwm_led_fade/     # Bring-up phase
│   │       └── mecanum_drive/    # Active project: full 4-motor mecanum drive,
│   │                             # 4-wheel quadrature encoder reading, and the
│   │                             # disarm-at-boot safety layer
│   ├── esp32/
│   ├── arduino/
│   └── drivers/
├── ros2_ws/                # ROS 2 Jazzy colcon workspace (runs on the Raspberry Pi 5)
│   └── src/
│       ├── mecanum_bringup/      # Launch files and config, including headless
│       │                         # Gazebo Harmonic simulation bring-up
│       ├── mecanum_description/  # Robot URDF (xacro) + display.launch.py
│       ├── mecanum_interfaces/   # Custom msg/srv/action definitions
│       └── mecanum_control/      # mecanum_kinematics and mecanum_odometry nodes
├── hardware/               # Hardware documentation
│   ├── cad/                # CAD files and 3D models
│   ├── schematics/         # Electrical schematics
│   ├── bom/                # Bill of materials
│   └── datasheets/         # Component datasheets
├── docs/                   # Documentation
│   ├── user-guide/         # User guides and development log
│   ├── videos/             # Demo videos
│   │   └── stm32/          # STM32 phase test recordings
│   └── images/             # Documentation images
│       ├── stm32/          # STM32 phase photos
│       └── hardware/       # Hardware build photos
├── config/
├── tests/
└── scripts/
```

---

## System Architecture

The project splits into two independently-running parts that will talk to each other over a serial link:

- **`firmware/`** — runs on the STM32 Nucleo-F446RE. Handles the real-time layer: 8-channel PWM generation for all four motors, quadrature encoder reading and velocity computation for all four wheels, and the disarm-at-boot safety gate. Once Phase F7 lands, it will also own the USART2 serial link to the Raspberry Pi.
- **`ros2_ws/`** — a ROS 2 Jazzy colcon workspace that runs on the Raspberry Pi 5. Currently holds four packages:
  - `mecanum_bringup` — launch files and configuration, including the headless Gazebo Harmonic simulation bring-up.
  - `mecanum_description` — the robot's URDF, built from measurements taken off the physical robot. `base_link` sits at ground level, at the centroid of the 200×200 mm wheel-contact square. `display.launch.py` brings up `robot_state_publisher` and `joint_state_publisher` for visualizing the model.
  - `mecanum_interfaces` — custom interfaces (`WheelSpeeds.msg`, `ResetOdometry.srv`, `MoveForSeconds.action`) shared between nodes.
  - `mecanum_control` — the `mecanum_kinematics` node (inverse kinematics, `/cmd_vel` → `/wheel_speeds`) and the `mecanum_odometry` node (forward kinematics, dead-reckoning odometry, `/odom` and the `odom`→`base_link` TF). Both are validated in a headless Gazebo Harmonic simulation via the `ros_gz` bridge, to 9 significant figures against the simulator, and visualized in Foxglove.

  `ros2_ws/` has its own `.gitignore` for `build/`, `install/`, and `log/`. An earlier workspace, `mecanum_actions_ws`, is retained only as a ROS 2 fundamentals learning archive — it is not part of the active system.

---

## Mecanum Wheel Kinematics

The mecanum wheel configuration allows for omnidirectional movement through independent control of each wheel's velocity.

| Movement      | FL  | FR  | RL  | RR  |
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
- **Duty cycle**: 0–999 steps (1000 resolution levels), capped at 300 during development by the safety layer
- **PWM frequency**: 1 kHz for development, 20 kHz for production (reduces audible noise)
- **Timer source**: TIM3/TIM4 on APB1 at 90 MHz, prescaler 89 (1 MHz tick), ARR/period 999, giving 1 kHz PWM

---

## Getting Started

### Prerequisites

- [STM32CubeIDE 2.0](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeMX 6.16.1](https://www.st.com/en/development-tools/stm32cubemx.html)
- ST-LINK Server (required on macOS for flashing)
- STM32Cube FW_F4 V1.28.3 firmware package
- USB cable for ST-LINK/V2-1 on-board debugger
- ROS 2 Jazzy + Gazebo Harmonic (on the Raspberry Pi 5, for the `ros2_ws/` side)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/KamalaIssack/Mecanum-wheel-mobile-robot-project.git
   ```

2. **Firmware:** Open STM32CubeIDE and import the `mecanum_drive` project from `firmware/stm32/nucleo_f446re/`. Use STM32CubeMX to regenerate peripheral configuration code if needed, then build and flash to the Nucleo-F446RE via the on-board ST-LINK debugger.

3. **ROS 2 (Raspberry Pi 5):**
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

### Development Workflow

1. Configure peripherals in STM32CubeMX (`.ioc` file)
2. Generate code (placed within `/* USER CODE BEGIN/END */` blocks to survive regeneration)
3. Build in STM32CubeIDE
4. Flash via ST-LINK

---

## Working Practices

- **Star topology**: Every power consumer connects directly to the bus bars, never daisy-chained
- **Voltage verification**: A multimeter reading is treated as ground truth for buck output voltage — not a buck module's onboard display — checked after any adjustment before powering compute boards
- **Arm before motion**: The board always boots disarmed; motors only receive PWM after an explicit `motors_arm()` call, and the development duty-cycle ceiling stays in place until closed-loop control is validated
- **Battery safety**: LiPo never left unattended while charging; inline fuse is the first thing in the positive line
- **User code protection**: All STM32 application code lives inside `/* USER CODE BEGIN/END */` blocks

---

## Technical Notes

- The system uses 8 PWM channels (2 per motor × 4 motors) across TIM3/TIM4
- ROS 2 workspace lives in this repo at `ros2_ws/`. The companion repository `mecanum_actions_ws` is now a ROS 2 fundamentals learning archive and no longer holds production packages.

For detailed firmware development notes, see [`docs/user-guide/stm32-development-log.md`](docs/user-guide/stm32-development-log.md).

---

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
