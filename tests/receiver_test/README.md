# Testing the RC receiver readout

This test is part of the ChIMP sketch.
Once the sketch is uploaded and running (robot propped up, wheels spinning freely), open the Serial Monitor from the Arduino IDE. 
Type h and ENTER to see availbe commands.
Using the respective commands, disable the motion controller, then enable PWM printouts.
As you move the sticks on the RC transmitter, you should see the three channels change their values accordingly. 
They represent the three signals’ pulse widths in microseconds and should be centered roughly around 1500, ranging from 1000 to 2000.
, 
