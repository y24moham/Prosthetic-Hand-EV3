# EV3 Prosthetic Hand (LEGO Mindstorms)

A prototype prosthetic hand built using **LEGO Mindstorms EV3**, focused on demonstrating a functional, motor-driven grip mechanism and a simple control system that can be tested and iterated quickly.

## Project Goals
- Build a **working prosthetic-hand-style gripper** using EV3-compatible components
- Implement reliable **open/close control**, with basic safety limits
- Explore **sensor-driven behavior** (optional), like grip triggers or feedback
- Document the design so it can be reproduced or improved

## Demo
- Video / photos: *(add link or put media in `/docs`)*

## Features
- Motor-driven hand actuation (open/close grip)
- Configurable control logic (manual or sensor-driven)
- Calibration approach (recommended) to prevent overdriving the mechanism
- Modular design intended for iteration

## Hardware Used
- **LEGO Mindstorms EV3 Brick**
- EV3 motor(s): *(Large / Medium motor — specify what you used)*
- EV3 sensor(s) (optional): *(Touch / Ultrasonic / Color / Gyro — specify)*
- Technic beams, axles, gears, linkages for the hand mechanism
- Cables, connectors, mounting parts

> If you want this repo to be reproducible, add a simple BOM in `docs/BOM.md`.

## Software / Environment
- EV3 programming environment: *(EV3-G, EV3 Classroom, RobotC, leJOS, Python EV3Dev — specify what you used)*
- OS (if applicable): *(Windows/macOS/Linux)*

## How It Works (High Level)
1. The EV3 motor actuates a linkage/gear mechanism to open/close the hand.
2. A control routine sets the motor direction/speed based on:
   - manual input (button / program toggle), and/or
   - sensor input (e.g., touch triggers grip), if used.
3. Optional calibration/limits prevent excessive torque and mechanical stalls.

## Setup
1. Assemble the prosthetic hand mechanism.
2. Connect the motor to the EV3 brick (port: `A/B/C/D`).
3. Connect any sensor(s) to the EV3 brick (port: `1/2/3/4`).
4. Upload/run the program.

## Usage
- **Open/Close**: *(describe your controls: button press, sensor trigger, timed routine, etc.)*
- **Grip strength/speed**: adjust motor power in code or project settings.
- **Calibration (recommended)**:
  - Run the hand to the open limit gently
  - Reset motor position (if supported in your environment)
  - Use a safe range for closing to avoid stalling or breaking parts

## Repo Structure (recommended)
