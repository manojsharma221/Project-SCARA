# Project-SCARA
A 3 DOF robotic arm with vision capability

Project Abstract: https://drive.google.com/file/d/1kxg55BfkzvxQh5KZDi9VbZtgp_u2S0rz/view?usp=sharing

Project Report  : https://drive.google.com/file/d/11Dyuc8OJAZLYCGUqu4pYYwd3xX3RIzmn/view?usp=sharing

Project DEMO Video: https://drive.google.com/file/d/11juNTGngP_k0XdUEG3fVxlW-YacUFE9F/view?usp=sharing

"SCARA_interrupt_coordinated.ino" is the micro-controller code. An arduinoMEGA, coupled with CNC Shield and A4988 stepper motor driver was used.

"SCARA_Inv_Kin.py" is a python file to manually control the robot.Here, the inverse kinematics equation of the manipulator is solved and using "PyCMDmessenger" library the arduino
is instructed, over a usb connection, to move the arm to the desired location.

The folder "SCARA Vision" contains the full-fledged robot code with image processing.By running "RoboSee.py", the robot will detect shapes and colors of objects,using a webcam, in its workspace and will move it's end-effector over each object one-by-one. This file was developed just to demonstrate the vision capability and automation of the project. 
