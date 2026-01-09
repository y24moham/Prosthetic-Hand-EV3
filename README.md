# EV3 Prosthetic Hand (LEGO Mindstorms)

A prototype prosthetic hand built using **LEGO Mindstorms EV3**, demonstrating a functional motor-driven grip mechanism and a control system designed for fast iteration and repeatable testing.

## Tech Stack
- **Platform:** LEGO Mindstorms EV3  
- **Programming:** **RobotC** (C-based)  
- **Controller:** EV3 Brick  
- **Actuation:** **3 EV3 motors** (Large + Medium) driving finger and thumb motion  
- **Sensors:** **Touch**, **Ultrasonic**, **Accelerometer**  
- **Mechanical:** LEGO Technic beams, axles, gears, linkages (hand structure + grip transmission)  
- **CAD / Fabrication:** **SolidWorks**, **3D printing (Prusa MK4, PLA)**  

## Project Goals
- Build a working prosthetic-hand-style gripper that can be developed toward a medical assistive concept, using EV3-compatible components
- Implement reliable open/close control with basic safety limits (encoder-based positioning)
- Explore sensor-driven behaviors (grip trigger, proximity-based actions, release conditions)
- Document the design so it can be reproduced or improved

## Demo
Demo folder: https://drive.google.com/drive/folders/1JLLC8HWLDYezZzPMAi1FyZIcRnsr5J4P?usp=drive_link
- Demo 1: https://drive.google.com/file/d/1u6iXG5a_sigPQkx7iz7OC_ltrGPNhpzM/view?usp=sharing  
- Demo 2: https://drive.google.com/file/d/1UtOttGSHC_REruDxqQpYOA-3Q0pk9Jdj/view?usp=sharing  
- Demo 3: https://drive.google.com/file/d/1OvK0Yru-BhYl1_SVyntyIL9PVSVa64wP/view?usp=sharing  
- Demo 4: https://drive.google.com/file/d/134nNxhyoesffMwnrE6gtcnD41NbJrs_N/view?usp=sharing  
- Touch sensor mechanism demo: https://drive.google.com/file/d/1osuuIexqot-xtJVTjMkOZRjLW-lSD0nd/view?usp=sharing  

## What We Built
- A 3-motor EV3 hand mechanism: two main fingers driven by large motors, plus a thumb driven by a medium motor through a geared linkage
- A compact sensor layout:
  - **Touch sensor** triggered via a palm “touch pad” linkage
  - **Ultrasonic sensor** positioned near the palm for proximity-based behavior
  - **Accelerometer** mounted on the back of the hand for motion-based logic
- A custom handle assembly designed in **SolidWorks** and **3D printed** to improve usability and allow comfortable operation

## Mechanical Design Highlights
### Thumb drivetrain (Medium Motor)
- The thumb is driven indirectly using **gears and axles** rather than direct drive.
- Multiple gear ratios were tested; a **small gear on the motor driving a larger gear** improved torque/stability (at the cost of speed).
- The thumb structure uses beams/pins similar to the main fingers, with an inverted tire for improved grip.

### Touch pad mechanism
- A low-profile palm pad pivots when pressed and transfers force to the EV3 **touch sensor**.
- This linkage reduces required press force and improves consistent activation from different pad locations.

### Ultrasonic placement
- The ultrasonic sensor is angled and positioned near the palm to detect objects ahead of the hand.
- In the implemented behavior, if an object is within **~20 cm**, the hand opens.

### Accelerometer placement
- Mounted on the back of the hand to measure motion in 3 axes and support “release” logic.
- Sensor orientation matters; the axis directions were accounted for in the program.

## 3D Printed Parts (SolidWorks + Prusa MK4)
- The handle consists of **three individually 3D printed parts** (printed on a **Prusa MK4**) and **glued** into one assembly.
- Designed in **SolidWorks** with comfort in mind (convex grip shape).
- The handle is printed in **PLA**; future iterations could add rubber or traction material to reduce slipping.

## Assembly Notes
- LEGO pins/axles were used throughout, but the overall chassis weight required extra reinforcement:
  - **Zip ties** were used to secure the chassis, motors, and ultrasonic sensor where friction/pins were insufficient.
  - The EV3 brick is mounted on the handle and held using **Velcro** for easy removal and access to buttons.

## Software Overview (RobotC)
- The program uses motor encoders to keep finger positions repeatable between runs.
- A dedicated initialization routine “zeros” the fingers so open/close behavior is consistent across trials.
- Motor helper routines support controlling individual motors or “ALL” motors together (with thumb speed adjusted relative to the fingers when closing).

## Setup
1. Assemble the EV3 prosthetic hand and handle.
2. Connect motors to EV3 ports (`A/B/C/D`) and sensors to ports (`1/2/3/4`) based on your build.
3. Upload the RobotC program to the EV3 brick.
4. Run the program and verify:
   - initialization completes correctly (fingers zero properly)
   - sensor triggers behave as expected
   - open/close limits avoid stalling the mechanism
