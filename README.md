# Groundbot Low-Level Firmware

This directory contains the firmware for the **Teensy 4.1** microcontroller used on the Agribots Ground Robot. It utilizes **micro-ROS** to communicate natively with the ROS 2 graph over Serial (USB).

## ⚙️ Hardware
* **MCU:** Teensy 4.1
* **Communication:** USB Serial (`/dev/ttyACM0`)
* **Key Peripherals:**
    * Motor Controllers (PWM/Direction)
    * Encoders (Interrupts)
    * IMU (If connected via SPI/I2C to Teensy)

## 💻 Development Environment

We recommend using **PlatformIO** (VS Code Extension) for building and uploading code, as it handles the micro-ROS library dependencies automatically.

### Prerequisites
1.  **VS Code** installed.
2.  **PlatformIO IDE** extension installed.
3.  **udev rules** installed (for Teensy upload permissions).

## 🔨 Build & Upload

1.  Open this folder (`microROS_package`) in VS Code.
2.  PlatformIO should automatically detect the `platformio.ini` file and install libraries.
3.  Connect the Teensy via USB.
4.  Run the **Upload** task (Arrow icon in bottom toolbar).

## 🔌 Connecting to ROS 2

The Teensy will not function until the **Micro-ROS Agent** is running on the host computer (Raspberry Pi).