# ELEGOO Smart Robot Car V4.0 – Firmware Installation Guide
**University Engineering Lab Setup & Deployment Manual**

This repository contains the official firmware and deployment instructions for the **ELEGOO Smart Robot Car Kit V4.0** (TB6612 Motor Driver & MPU6050 Gyroscope chipset). This guide is designed to help students correctly flash the microcontroller board and ensure the correct workspace environment is configured.

---

## 📋 System Requirements & Hardware Prep
1. **Microcontroller:** ELEGOO UNO R3 (ATmega328P).
2. **Hardware Toggle:** Before attempting any upload, you **must** slide the physical **UPLOAD/CAM switch** located on the car's mainboard to the **"UPLOAD"** position. Failure to do so will cause communication timeout errors.
3. **Connection:** Connect the board to your workstation via a standard USB-B to USB-A cable.

---

## 🚀 Method 1: Pre-Compiled Deployment (Recommended for Quick Lab Setup)
*Bypasses local IDE compiler environments and avoids library version mismatches entirely.*

### Windows Workstations (Using XLoader)
1. Download and extract **XLoader** utility.
2. Locate the pre-compiled `.hex` firmware file within this repository package.
3. Launch `XLoader.exe` and configure the following parameters:
   - **Hex File:** Browse and select your `SmartRobotCarV4.0.hex` file.
   - **Device:** Select `Uno(ATmega328P)` from the dropdown menu.
   - **COM Port:** Identify the active port assigned to your car via *Windows Device Manager -> Ports (COM & LPT)*.
   - **Baud Rate:** Set strictly to `115200`.
4. Ensure the car's physical upload switch is on, then click **Upload**. Successful flashing will be indicated by rapid blinking of the Tx/Rx LEDs on the board.

### macOS & Linux Workstations (Using AVRDUDESS)
1. Launch **AVRDUDESS** (GUI wrapper for `avrdude`).
2. Set **Programmer** to `Arduino` (or `stk500v1`).
3. Set **MCU** to `ATmega328P`.
4. Set your active serial port and configure the speed baud rate to `115200`.
5. Under the **Flash** field, select your `.hex` file, choose format `Intel Hex`, and click **Go**.

---

## 🛠️ Method 2: Compiling Code via Arduino IDE 2.x
*Required for students modifying behavior algorithms or completing code-based lab assignments.*

### Target File
Students must navigate to the main program directory and open:
`02 Main Program / TB6612_MPU6050 / SmartRobotCarV4.0.ino`

### ⚠️ Critical Environment Note: FastLED Library Dependency
To compile the original code without dependency errors, **do not** download the newest FastLED release via the Library Manager. Instead, use the exact legacy FastLED library files included directly in ELEGOO's original source code software package folder. 

If compilation fails due to an external library conflict, ensure your local workspace directories are cleanly using the source folder's bundled assets rather than modern global package overrides.

---

## 🎯 Verification Checklist
After a successful installation:
* Return the physical board switch to the **"CAM"** position if you intend to stream live video.
* Power on the car utilizing the onboard battery pack.
* The ultrasonic servo turret should perform an initialization sweep, indicating correct firmware deployment and execution.
