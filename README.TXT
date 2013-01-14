README

This is a slightly modified version of the FreeIMU code from http://www.varesano.net/projects/hardware/FreeIMU 

The mods are to FreeIMU_serial/FreeIMU_serial.ino, and add a default output of compass heading, yaw,roll,and pitch in a useful format for FreeBoard. 
The FreeboardIMU sketch is the same as FreeIMU_serial.ino, but without the ability to use the FreeIMU calibration routime. Basically it does not respond to commands, which were causing problems when Freeboard sent data to the IMU.

Use the Arduino IDE 1.02, and open the FreeIMU_serial project. Build and deploy to your ArduIMU v3. Then calibrate. Then delpoy the FreeBoardIMU project to the board before you use it in FreeBoard.

***The ArduIMU will NEED calibration to provide stable output. Believe me there is no point in struggling on without doing the calibration (as per the FreeIMU site), you just get rubbish output and waste weeks (!*&%^$!).
