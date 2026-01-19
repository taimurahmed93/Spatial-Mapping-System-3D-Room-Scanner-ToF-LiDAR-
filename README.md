# Spatial-Mapping-System-3D-Room-Scanner-ToF-LiDAR-

A low-cost embedded **3D spatial mapping / room scanning system** that builds a point-cloud model of indoor environments using a **VL53L1X Time-of-Flight sensor** mounted on a **stepper motor**. Distance measurements are captured across **360°** at fixed angle increments and streamed to a PC over **UART**, where the data is reconstructed and visualized in 3D using **Python + Open3D**.

> Built for COMPENG 2DX3 (Microprocessor Systems Project) – McMaster University.

---

## Demo / Output
The system produces a 3D reconstruction (wireframe / point cloud surface) of scanned indoor hallways/rooms based on multiple scans along the x-axis (fixed displacement between scans).

✅ Example scan outputs and comparisons are included in the project report.

---

## Key Features
- 360° planar scanning using **stepper motor rotation**
- **Distance sampling every 11.25°** (512-step motor rotation)
- **I2C** interface (ToF → microcontroller)
- **UART** streaming (microcontroller → PC)
- **Python pipeline**: parse serial data → convert polar → XYZ → render 3D model
- Status feedback using **on-board LEDs**
- Manual “multi-scan” workflow for 3D mapping (x-axis displacement between scans)

---

## Hardware Used
- **MCU:** MSP432E401Y (Cortex-M4F + FPU)
- **Distance Sensor:** VL53L1X Time-of-Flight (up to ~4m range)
- **Motor:** 28BYJ-48 Stepper Motor (512 steps / revolution)
- **Motor Driver:** ULN2003 driver board

---

## System Overview (How it works)
1. Press **PJ0** to begin a scan.
2. Stepper rotates 360° and the system samples distance at fixed increments.
3. Each reading is sent via **I2C** from the VL53L1X to the MSP432.
4. MSP432 streams the data to a PC using **UART**.
5. PC-side Python scripts:
   - read serial stream (PySerial)
   - convert distances + angles into **XYZ coordinates**
   - render using **Open3D** for a full 3D visualization

A simplified data flow:
**VL53L1X → (I2C) → MSP432 → (UART) → PC Python → Open3D Visualization**

---

## Software / Tools
- Embedded:
  - Keil uVision (MSP432 development)
  - I2C + UART firmware
- PC:
  - Python
  - PySerial
  - NumPy
  - Open3D

---

## How to Run
### Embedded setup
1. Wire the circuit according to the schematic in the report.
2. Build + flash firmware to MSP432E401Y in Keil.
3. Reset the board.

### PC visualization
1. Identify COM port in Device Manager.
2. Update the COM port in the Python serial script.
3. Run the Python transfer/record script (collect scans).
4. Run the Open3D script to render the final 3D model.

---

## Results / Notes
- The scan accurately captures hallway structure and geometry.
- Trig/coordinate conversion is better handled on PC for speed and higher precision.

---

## Repository Structure
- C files are under Keil Code
- Python files are under Python Code


