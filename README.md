
# Project TAMIYA

GitHub repository for custom open-source RC car system based on STM32 microcontrollers. The project is built on top of an old Tamiya QD chassis, which was modified. The repository includes both car and transmitter software along with hardware schematics and the Bachelor's thesis with extensive description of the project.


## Features

- long range (350 m+)
- high speed (35 km/h)
- smooth control (~220 Hz control loop)
- 2.4 GHz two-way communication 
- available driving data:
	- battery voltage and current
	- BLDC shell temperature
	- linear acceleration and angular velocity
	- supports logging to microSD card


## Car

- STM32F303CCT6 MCU
- nRF24L01+ PA + LNA module
- RC servo Emax ES3001
- GoolRC 2435 BLDC 4800Kv with 25A ESC
- OLED display and RGB LED
- ACS712 hall current sensor
- DS18B20 temperature sensor
- MPU6050 IMU


## Transmitter

- STM32F303CCT6 MCU
- nRF24L01+ PA + LNA module
- OLED display and RGB LED
- Custom PCB

