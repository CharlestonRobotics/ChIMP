# Automated ODrive setup for ChIMP
Once you installed all tools following these instructions https://docs.odriverobotics.com/#downloading-and-installing-tools this script takes care of setting up the ODrive. This will only work if the wiring of the motors and Hall sensors is correct.

All you need to do is 
1. Wire up the ODrive and the motors
2. Power up the ODrive
3. Make sure both motors are ABSOLUTELY FREE TO TURN (put the robot on a stand)
4. Connect the ODrive to your computer via USB
5. Run the script using a python3 interpreter. On Linux this will look like so for motor 0
```console
./odrive_setup.py -a 0
```
and like so for motor 1
```console
./odrive_setup.py -a 1
``` 
