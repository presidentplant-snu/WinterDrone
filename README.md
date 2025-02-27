# Winter Drone Project

A small winter-break project of creating a small drone with an RPi 2 Pico W controller.

Made with Raspberry Pi Pico 2W, MPU6050, BMP280, two dual n-channel mosfets and 4 coreless motors.

Since the wifi (PicoTesting) doesn't have a DHCP server yet, one must connect via static IP. The IP of the pico is 192.168.4.1

## TODO
- Implement sensor fusion between IMU (MPU6050) and Barometer (BMP280)
- Add DHCP server
