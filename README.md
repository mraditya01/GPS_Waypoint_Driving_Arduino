# GPS_Waypoint_Driving_Arduino
## Table of contents
* [General info](#general-info)
* [Library](#library)
* [Module](#modules-used)

## General Info
<br> Arduino Basic Robotic Car for Waypoint Autonomous Navigation using GPS. The robotic car will move using bluetooth control into a GPS coordinate then save the coordinate waypoint, after three waypoint received, the car will be able to return to original position. <br>
This project is aimed for me to understand more about robotic car, Arduino, and it's modules after trying the ultrasonic obstacle avoiders. Compass QMC5883l is used in this project, which will give the current heading to be able to calculate the heading error by obtaining the waypoint azimuth and the robotic car’s heading with respect to the North, obtained from the digital compass.

## Library
- TinyGPS++
- SoftwareSerial
- Wire
- QMC5883LCompass

## Modules Used
1. Ren He NEO-6M GPS Module Active Antenna
   https://www.amazon.co.jp/gp/product/B07LDX31FY/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
2. GY-271 QMC5883L 3-Axis Magnetic Sensor
   https://www.amazon.co.jp/gp/product/B075N8Z4FC/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1
3. OSOYOO Model 3 V2.0 Arduino DIY Robot Car Kit

