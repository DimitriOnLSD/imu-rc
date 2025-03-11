![alt text](https://github.com/DimitriOnLSD/imu-rc/blob/main/assets/images/render_main.png)
# IMU-RC
This repository contains the final project for an Electronics and Computer Engineering degree: IMU-RC, an innovative hand-held controller for RC cars.

## Features
### Hand Unit
The hand unit is a battery-powered controller featuring:

- ESP32-S3 WROOM module on a custom PCB designed in KiCad.
- LSM6DSO IMU sensor for motion detection.
- BLE communication using a client/server protocol with the RC car.
- OLED screen (128x32) for menu options, diagnostics, and sensitivity adjustments.
- Four buttons for navigation: Back, Up, Select, Down.
- RGB LED to display battery or communication status.
- Battery Management System (BMS) with BQ297 and MCP73831 to charge a 3.7V 250mAh LiPo battery via USB-C.
- BOOT and RESET buttons for easy firmware handling.

### Gesture-Based Control
Control the RC car with intuitive hand gestures:

- Lean forward: Car moves forward.
- Lean backward: Car reverses.
- Lean left or right: Car rotates left or right.
- Control mode toggle: For 4-motor vehicles, enable sideways movement for strafing.

### Additional Features
- Car stats menu: Check car battery levels.
- Safety mechanism: Proximity sensors (VL53LX) prevent unwanted motion:
  - 2-motor cars: Front and back sensors.
  - 4-motor cars: Side sensors added for additional safety during strafing.
  
### Technical Highlights
- Custom PCB: Designed in KiCad for compact, efficient performance.
- Battery Management: MCP73831 handles charging, and BQ297 ensures protection against overcharging/discharging.
- Versatile Control: Adaptable to different RC car configurations (2-motor or 4-motor).
- OLED Menu System: Offers detailed settings, diagnostics, and real-time feedback.

### Usage
IMU-RC is designed to bring modern, intuitive control to RC vehicles, making it a perfect blend of technology and engineering creativity.
