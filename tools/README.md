# Automated ODrive setup for ChIMP
Once you installed all tools following these instructions https://docs.odriverobotics.com/#downloading-and-installing-tools, this script takes care of setting up the ODrive for ChIMP. This will only succeed if the wiring of the motors and Hall sensors is correct - if it is not, setup will fail and the script will tell you what is wrong.

If you have already fully built your ChIMP, ***make sure the Arduino is powerd off for this test***.

All you need to do is 
1. Wire up the ODrive and the motors
2. Make sure both motors are ABSOLUTELY FREE TO TURN (put the robot on a stand)
3. Connect the ODrive to your computer via USB
4. Power up the ODrive
5. In a system terminal, navigate to the ChIMP/tools folder.
6. Run the script using a python3 interpreter. On Linux this will look like so for motor 0
```console
./odrive_setup.py -a 0
```
and like so for motor 1
```console
./odrive_setup.py -a 1
``` 
