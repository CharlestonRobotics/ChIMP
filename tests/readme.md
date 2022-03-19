ChIMP is not hugely complex, but complex enough that it makes sense to test things along the way to prevent "big bang integration".

***For all these tests, make sure the robot is secured in place and the wheels are free to turn!***

# Testing ODrive and motors
Once the ODrive is connected to the motors and Hall sensors, it's a good idea to test that it can make the motors spin. *If you have already fully built your ChIMP, make sure the Arduino is powerd off for this test.*

Here are the steps:

* Follow the steps in the [tools/readme](https://github.com/CharlestonRobotics/ChIMP/blob/master/tools) to configure the ODrive. You should see the motors sping during calibration. If that's the case, go to the next step. If not, you are in for some troubleshooting. 
* With the ODrive powered and connected to your computer via USB, launch the [odrivetool](https://docs.odriverobotics.com/v/latest/getting-started.html#start-odrivetool).
* Enable motor 0 (in ODrive speak, axis0): 

``odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL``
* Send a few current commands to your motor. Start small (e.g. 500mA) and see at which current iTht starts spinning:

``odrv0.axis0.controller.input_torque = 0.5``
* Disable motor 0: 

``odrv0.axis0.requested_state = AXIS_STATE_IDLE``
* Do the same for motor 1 (aka axis1).

Both motors should behave roughly the same for the same amount of current. 
*If you see the script terminating with encoder errors, you can run the Hall test.*

# Testing the Hall encoders without the ODrive
For this test, wire the Hall sensors directly to the Arduino. The red wire goes to 5V and the black to GND.
The other three wires (signals A, B and C) will be connected to the Ardunino pins A0, A1 and A2 - the order does not matter.

* Connect the Arduino to your computer and load the hall_test.ino sketch.
* Open the serial terminal. You will see 5 collmuns of numbers:
The first three numbers make up the current hall sensor state. 
The fourth number is the counter of valid state transitions. On a healthy motor, this should increase by exactly 90 per rotation.
The fifth number is the counter of invalid or 'illegal' state transitions. This should always be 0. 
* If the fifth number ever goes beyond 0, something is wrong with the Hall encoders and the ODrive won't be able to calibrate properly.
* You can reset the counters by typing ```r``` in the serial terminal and hitting ENTER. 

# Testing Arduino, ODrive and motors
Hopefully the previous test went well. If so, let's put the Arduino in the loop!

* Connect the Arduino to the ODrive via their serial interface (Serial2 on the Arduino, UART_A on the ODrive).
* Load the [ChIMP/tests/motor_test/motor_test.ino](https://github.com/CharlestonRobotics/ChIMP/tree/master/tests/motor_test) file to your Arduino and open the Arduino serial terminal (@115200bps)
* Power up the ODrive.
* In the serial terminal, type h and hit ENTER
* Enable the motors with the enable command
* Command somme current to one of the motors (say 500mA to motor 0 with 'c 0 0.5')
* Stop the motor with either of the stop commands (freewheel or disable)
* Try the other motor, and try both at the same time

You should see the same behaviour as in the previous test.

# Integration tests
With your chimp fully built up, powered, secured and it's wheels to turn, it's time for some integration testing!

For these tests, load the [ChIMP.ino](https://github.com/CharlestonRobotics/ChIMP) sketch to your Arduino. 
Before each test, power off the robot and power it on, so that you start from a repeatable 'clean' state.
After each test, power down the robot by disconnecting the battery.

## RC test

* Type h and hit ENTER in the serial console to show all available commands.
* Disable the motion controller
* Enable periodic PWM printouts. You should see three collumns of numbers being printed in the console.
* Move the sticks of your RC transmitter. You should see the three numbers change their values accordingly. 
They represent the three signals’ pulse widths in microseconds and should be centered roughly around 1500, ranging from 1000 to 2000.

## IMU test
* TODO(LuSeKa)

## RC control test
* With the respective command, disable the IMU.
* Using the enable input on your transmitter, enable the robot. *The wheels will likely start spinning at this point.*
* Now you should be able to control the wheels wih the sticks on your transmitter. 
* Whe throttle is applied the wheels should turn in the same direction, and steering should make them turn in opposite directions.
* TODO(LuSeKa): What to do if the wheels spin the wrong way?

## IMU motor control test
* With the respective command, disable RC control.
* Using the enable input on your transmitter, enable the robot. *The wheels will likely start spinning at this point.*
* If you tilt the robot, the wheels should react by spinning faster or slower.
* The wheels should spin such that they would make the robot drive in the direction it is leaning towards.
* TODO(LuSeKa): What to do if the wheels spin the wrong way?

If all these test pass, your ChIMP might be ready to roll!

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
* TODO(LuSeKa)
