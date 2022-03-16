# Testing the ODrive and remote control without the IMU

This test is part of the ChIMP sketch.
It is inteded to make sure the Arduiono communicates with the ODrive, the motors can be controlled via the remote control and the turn in the correct directions.
Make sure the robot's wheels are not touching the ground.
Slide the activation switch on the remote to the low position, then power-up the robot.
Open a serial terminal, type h and ENTER to see possible commands.
Using the respective command, disable the IMU. 
Once you slide the activation switch, the motors might start spinning.
When applying throttle, the wheels should turn in the same direction. 
When applying steering, they should move in opposite directions. 
These two commands are superimposed.


