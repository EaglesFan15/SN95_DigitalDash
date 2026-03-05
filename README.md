SN95 Mustang Digital Dashboard for use with Holley Terminator X ECU

Project HardWare:
Waveshare ESP32-S3 5in Touch LCD
Arduino Nano ESP32
Arduino Nano ESP32 Breakout Board
4 1.28in OLED Screens 
Custom PCB (KiCAD Files attatched)
Molex KK254 Connectors
NEO 6M GPS Module w/Antenna
CAN Transiever
3d Printed Instrument Cluster (STL and STEP attatched)

The Dashboard consists of two separate ESP32s, I used the Arduino Nano ESP32 because I had it on hand. The seperate sketches can be configured to display whatever parameters you want given you have the Holley CAN IDs for the parameter. 

PINOUTS:

Waveshare ESP32-S3 5in Touch LCD:
CAN HI- 
CAN LO-
CAN GND
GPS RX-
GPS TX-

ARDUINO NANO ESP32:


Based on Waveshare LVGL Porting Example for Waveshare ESP32-S3 5in Touch LCD

This sketch and accompanying files will only work with a Waveshare ESP32-S3 5in Touch LCD
Must have LVGL (v8.3.x) installed. 



