ChIMP is not hugely complex, but complex enough that it makes sense to test things along the way to prevent "big bang integration".

***For all these tests, make sure the robot is secured in place and the wheels are free to turn!***

# Testing ODrive and motors
Once the ODrive is connected to the motors and Hall sensors, it's a good idea to test that it can make the motors spin.
*If you have already fully built your ChIMP, make sure the Arduino is powerd off for this test.*

Here are the steps:

* Follow the steps in the [tools/readme](https://github.com/CharlestonRobotics/ChIMP/blob/master/tools) to configure the ODrive.
You should see the motors sping during calibration. If that's the case, go to the next step. If not, you are in for some troubleshooting.
*If you see the script terminating with encoder errors, you can run the Hall test.*
* With the ODrive powered and connected to your computer via USB, launch the [odrivetool](https://docs.odriverobotics.com/v/latest/getting-started.html#start-odrivetool).
* Enable motor 0 (in ODrive speak, axis0): 

``odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL``
* Send a few current commands to your motor. Start small (e.g. 500mA) and see at which current it starts spinning:

``odrv0.axis0.controller.input_torque = 0.5``
* Disable motor 0: 

``odrv0.axis0.requested_state = AXIS_STATE_IDLE``
* Do the same for motor 1 (aka axis1).

Both motors should behave roughly the same for the same amount of current. 

# Testing the Hall encoders without the ODrive
*If you were able to spin the motors with the ODrive there is no need to rund this test.*


For this test, wire the Hall sensors directly to the Arduino. The red wire goes to 5V and the black to GND.
The other three wires (signals A, B and Z) will be connected to the Ardunino pins A0, A1 and A2 - the order does not matter.

* Connect the Arduino to your computer and load the [hall_test.ino sketch](https://github.com/CharlestonRobotics/ChIMP/blob/master/tests/hall_test/hall_test.ino).
* Open the serial terminal. You will see 5 collumns of numbers:
The first three numbers make up the current hall sensor state. 
The fourth number is the counter of valid state transitions. On a healthy motor, this should increase by exactly 90 per rotation
(note that it increases independent of the direction the wheel turns).
The fifth number is the counter of invalid or 'illegal' state transitions. This should always be 0. 
* If the fifth number ever goes beyond 0, something is wrong with the Hall encoders and the ODrive won't be able to calibrate properly.
A possible fix is the increase the capacity of the Hall filter capacitors from 22nF to 47nF.
* You can reset the counters by typing ```r``` in the serial terminal and hitting ENTER.

# Testing Arduino, ODrive and motors
If the ODrive setup and test went well it's tiime to put the Arduino in the loop!

* Connect the Arduino to the ODrive via their serial interface (Serial2 on the Arduino, UART_A on the ODrive).
* Load the [ChIMP/tests/motor_test/motor_test.ino](https://github.com/CharlestonRobotics/ChIMP/tree/master/tests/motor_test) file to your Arduino and open the Arduino serial terminal (@115200bps)
* Power up the ODrive.
* In the serial terminal, type h and hit ENTER
* Enable the motors with the enable command
* Command somme current to one of the motors (say 500mA to motor 0 with 'c 0 0.5')
* Stop the motor with either of the stop commands (freewheel or disable)
* Try the other motor, and try both at the same time

You should see the same behaviour as in the previous test whereyou commanded current through the ODrive tool.

# Integration tests
With your ChIMP fully built, powered, secured in place and its wheels free to turn, it's time for some integration testing!

For these tests, load the [ChIMP.ino](https://github.com/CharlestonRobotics/ChIMP) sketch to your Arduino. 
Before each test, power off the robot and power it on again, so that you start from a repeatable 'clean' state.
After each test, power down the robot by disconnecting the battery.

## RC test

* Type h and hit ENTER in the serial console to show all available commands.
* Disable the motion controller
* Enable periodic PWM printouts. You should see three collumns of numbers being printed in the console.
* Move the sticks of your RC transmitter. You should see the three numbers change their values accordingly.
They represent the three signals’ pulse widths in microseconds and should be centered roughly around 1500, ranging from 1000 to 2000.

## IMU test
* TODO(LuSeKa)

## IMU motor control test
* With the respective command, disable RC control.
* Using the enable input on your transmitter, enable the robot. *The wheels will likely start spinning at this point.*
* If you tilt the robot, the wheels should react by spinning faster or slower.
* The wheels should spin such that they would make the robot drive in the direction it is leaning towards.
* If one or both wheels spin in the wrong direction in response to the IMU, you can chang their direction individually
with the value of the kMotorDir0 and kMotorDir1 parameters in the Arduino Sketch. Change the value accordingly,
load the sketch on your Arduino and run this test again to verify that the motors behave correctly now. Repeat this step if neccessary.

## RC control test
* With the respective command, disable the IMU.
* Using the enable input on your transmitter, enable the robot. *The wheels will likely start spinning at this point.*
* Now you should be able to control the wheels wih the sticks on your transmitter.
* When throttle is applied the wheels should turn in the same direction. They should be turning in the opposite direction of the applied throttle:
For forward throttle, the wheels should turn backwards and vice versa. If that is not the case, change the value of the kThrottlePolarity parameter in the sketch
(or invert the signal if your tranmsitter makes that easy).
* The wheels should turn in opposite directions when you appy steering input. If you steer right, the left wheel should forward and the right wheel should turn backwards.
If that is not the case, change the value of the kSteeringPolarity parameter in the sketch
(or invert the signal if your tranmsitter makes that easy).

## Hall sensor calibration (for Arduino)
Each wheel has a set of three Hall sensors that are used by the ODrive to control the motor.
They are also used by the Arduino to keep track of the wheel position (odometry) and the robot's velocity.
Since the wires can be connected to the Arduino in any order, they need to be calibrated.
The calibration result (a sequence of six numbers) encodes all that the Arduino needs to know to make sense of the Hall sensor signals.
* With the robot propped up (wheels not touching anything) and the motors disabled, enter the command 'c 0' to begin Hall calibration for the left wheel.
* Manually turn the left wheel (slowly) forward until the serial console anonunces a successful calibration (only a few degrees are needed).
* Manually type the result (six numbers) into the curly braces of the left_hall_calibration parameter, separated by commas (replacing the numbers that are in there).
* Do the same for the right wheel (use command 'c 1').
* To apply the calibration, upload the ChIMP.ino sketch again.

If all these test pass and steps succeed, your ChIMP might be ready to roll!

# First time rolling
***THESE STEPS REQUIRE CAUTION! DO THIS IN A SPACIOUS AREA, WITH NOTHING FRAGILE ANYWHERE NEAR (INCLUDING PETS AND KIDS!). ALL KINDS OF THINGS COULD GO WRONG.***
* Disconnect all wires from your robot.
* Make sure the transmitter is turned on and the enalbe/disalbe input is in its disalbe position.
* Power on the robot.
* Put the robot on the floor, holding it upgright.
* Enable the motors via your transmitter. The robot should now be balancing if you let it go, slowly drifting away. Congratulations!!
* You should be able to drive it around with your transmitter.
* If it does not balance properly, over- or underreacts to your transmitter, starts vibrating, it's time for some parameter tuning.

# Parameter tuning 
The most important parameters to tune are *kpBalance* and *kdBalance*. They determine how the robot reacts to tilt and the rate of tilt, accordingly. You can change both 'live' via the serial interface. However, for now, your changes are not permantently saved (meaning your changes will be gone after a power cycle). After you found good values for these parameters, you need to change their values in the sketch accordingly and upload the sketch again to make your changes permament. 
* kpBalance: If this value is too small, the robot will lean too much and drift away quickly. If it it is too high, it will oscillate back and forth, maybe even violently. You want to find a value somewhere in between, the higher the better.
* kDBalance: If this value is set right it will cancel the oscillations that kpBalance can cause, allowing you to use a higher kpBalance (which is great). If it is set too high it will cause (potentially violent) relatively high frequency oscillations (more like a vibration). You want to find a value in between, the higher the better. 

Refer to comments in the code for tuning of the other parameters.
