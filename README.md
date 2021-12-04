# StarPointer
Telescope tracking software for the Arduino platform providing both a user interface and a machine interface for communicating with software packages such as Stellarium.
## Overview

This repository contains the source code for a telescope tracking device; an inertial measurement unit is attached to the telescope to measure where in the sky it is pointing. These measurements are then used to calculate the Right Ascension and Declination of the telescope which can then be read by an application such as Stellarium and presented to the user as a moving reticule on the sky simulation in Stellarium.

As such it provides a similar experience as a user with an astronomy app when they point their phone at the sky.

In addition to the IMU sensor a GPS sensor is used to determine the location of the observation and the time of the observation. A temperature and barometric pressure sensor is used to calculate corrections for the observed pointing direction, due to atmospheric diffraction.

A small TFT screen with attached mini joystick and buttons is also used to provide diagnostic feedback and to allow the user to read the data and measurements collected by the sensors.

## Hardware
The hardware is comprised of a microcontroller, a display/HID device and a number of sensor breakout boards (see parts list below). All of the sensors communicate with the microcontroller by means of an I2C bus. The display is connected to both the microcontroller's I2C bus as well as an SPI bus with a couple of digital pins for control. The presented implementation uses an Arduino MKR Wifi 1010 microcontroller but it could be replaced by a much smaller microcontroller board as long as it had the requisite connections and a processor at least equivalent to a SAMD Core M0. A lesser processor maybe capable of running the software,but without testing it would be difficult to say. No performance issues have been experienced running the software with the hardware listed.

### Components
The parts lists is split into essential and optional components. Essential components represent the minimum required to make the solution work as presented. 

However by making compromises in functionality and with some minor modifications to the code it is possible to omit the GPS sensor and display from the project completely. 

The GPS sensor can be omitted if the position of the telescope is hard coded into the software and there is some means to obtain the current time accurately. This could be either from an RTC component or given that the Arduino MKR Wifi 1010 has the ability to connect to a network then some network based time signal could be used; NTP servers

The display can be omitted since its only real purpose is to provide a simple feedback and diagnostics facility. The actual results from the hardware are displayed on the host desktop running an application such as Stellarium.

The only two really essential components are the microcontroller and the IMU sensor since the bare minimum requirement is to detect the pointing angle of the telescope and then convert that to a Right Ascension and Declination figure and present them to the host desktop in the correct format when the host software requests it.

The BNO055 IMU was selected primarily because the Adafruit technical details specifically state that it outputs absolute Euler angles and it was thought that it would be easier to implement the software with a sensor which outputs the Alt/Az values directly rather than quaternions which would need conversion to Alt/Az before calculating the Ra/Dec values. 

It should be possible to replace the BNO055 with the [Adafruit BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/) as the [conversion from quaternions to Euler values](http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/) is straightforward and appears to be within the capability of the processor in the context of the application. It may be possible to get the BNO085 to report absolute Euler values directly as Euler output is noted on the reverse of the breakout board however I may have misunderstood this as I haven't actually attempted the replacement/integration. 

All of the components can be replaced with similar devices provided that the requisite changes are made to the software with reference to the necessary interface libraries and code. 

#### Essential
* Microcontroller: [Arduino MKR Wifi 1010](https://docs.arduino.cc/hardware/mkr-wifi-1010)
* Inertial Measurement Unit [Adafruit 9-DoF BNO055 IMU](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
* GPS [Adafruit Mini GPS PA1010d](https://learn.adafruit.com/adafruit-mini-gps-pa1010d-module)
* Display with Joystick [Adafruit Mini Colour TFT with Joystick](https://learn.adafruit.com/using-circuitpython-displayio-with-a-tft-featherwing/mini-color-tft-with-joystick-featherwing)

#### Optional
* Temperature and Barometric Pressure Sensor: [Adafruit BMP390](https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx)  

The software uses the Temperature and Barometric pressure to calculate more accurately the variation, due to atmospheric diffraction, between the observed (pointed at) position and true position of the telescope. If omitted from the project then the software will use sensible defaults.

## Software
The software has been written using the Arduino IDE in C/C++

### Libraries
The code has the folowing dependencies these are standard Adafruit and Arduino libraries which can be imported using the Arduino IDE library manager.

* Adafruit_BMP3XX
* Adafruit_BNO055
* Adafruit_GPS
* RTCZero
* Adafruit_GFX
* Adafruit_ST7735

In addition to the interface libraries necessary for the hardware the software also has a dependency on the [AntelopeIT_Atrolib](https://github.com/Antelope-IT/AntelopeIT_Astrolib) library.
## How to Use

To be completed...

## To Do
* The project hardware built on breadboards and software has been run successfully on a bench both standalone and when connected to Stellarium running on a Raspberry Pi. The next stage is to house the hardware in a more permanent construction and to mount the IMU on the telescope.
* Complete the documentation to cover the current functionality.
* Add the Option for Wifi connectivity

## Health Warning
You are welcome to use what you see here and to learn from my mistakes but I make no guarantees regards the quality, functionality or suitability of this project for your application. You are free to use and adapt the information, ideas and designs contained in this project but you do so entirely at your own risk.
