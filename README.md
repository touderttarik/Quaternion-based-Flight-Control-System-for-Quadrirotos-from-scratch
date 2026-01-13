# Quaternion Based Flight Controller From Scratch

ESP-IDF project implementing a quaternion-based attitude estimation and control loop for a quadrotor.

## Current status
- MPU6050 I2C sensor readout (gyro/accel)
- Mahony AHRS filter for orientation
- Quaternion utilities and vector/matrix helpers
- PID angular rate controller (work in progress)
- Optional serial plotting helper (`tools/plot_serial.py`)

## Build (ESP-IDF)
1) Set up ESP-IDF and export the environment
2) Build/flash:

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Structure
- `components/orientationapi`: math, filters, sensors, PID, plotting
- `main/main.c`: app entry point and control loop
