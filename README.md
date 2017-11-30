# Quadcopter

Code for a quadcopter using an Arduino UNO as the controller.

Current setup:
Arduino UNO as main controller
Arduino Nano recieves RC values from a Turnigy 9x reciever and sends them over Serial at 115200 baud to the Arduino UNO. (This is because the arduino cannot be multithreaded and reading RC values needs to be done by reading pwm signals with interrupts but reading and writing i2c values to the IMU cannot be interrupted so the RC reading would have a very low frequency of getting new data if it was done on the UNO).
IMU: MPU-9250 (Sparkfun Breakout): using i2c communication
Sunfounder 16ch 12 bit servo driver: using i2c communication.  (This is needed since the arduino only has 8 bits of resolution on its pwm pins so this allows for more precise control.  Also, this allows the copter to be controlled with a Intel Edison or other board that is not ideal for servo control.)
EMAX RS2205-S 2300KV Motors
EMAX 30A DShot Bullet Series BLHeli-S ESCs
3D-Printed Frame: T4 Quadcopter Mini 250 Drone (5 inch props) https://www.thingiverse.com/thing:304237.
