README

This is a slightly modified version of the FreeIMU code from http://www.varesano.net/projects/hardware/FreeIMU 

The mods are to FreeIMU_serial/FreeIMU_serial.ino, and add a default output of compass heading,  yaw,roll,and pitch in a useful format for FreeBoard. It also outputs the equivalent NMEA sentences for other devices.

The FreeboardIMU sketch is basically the same as FreeIMU_serial.ino. It responds to different commands, to suit freeboards needs, and calibration process.

To use:

Download the FreeBoardIMU.cpp.hex above, install and calibrate with the freeboard-installer (https://github.com/rob42/freeboard-installer)

To develop:

Clone the project and use your favorite C/C++ editer to edit.

Use the Arduino IDE 1.52 to build and delpoy the FreeBoardIMU project to the ArduIMU. Then use the freeboard-installer to calibrate.

***The ArduIMU will NEED calibration to provide stable output. Believe me there is no point in struggling on without doing the calibration, you just get rubbish output and waste weeks (!*&%^$!).
