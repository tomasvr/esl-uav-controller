# To Do list

## High priority

- adjust joystick sensitivity (too high)
	- translate lift to appropriate values

- Define maximum difference between counteracting motors (400 300 400 300) max diff: 50
	- currently we divide by 320, but should use translate function instead

- extensively check safety features
	- panic mode: ESC button, js trigger, usb disconnect, battery value
	- Cannot enter manual/control mode unless js is neutral
	- Check all mode switches

- change while statement to if statement in pc terminal to limit amount of packets send shortly after each other

- trimming function using keyboard
	- counteract external offsets (e.g. cable)
	- offest value for setpoint of pitch, roll and yaw

- Time the duration of control loops, needs to be under 10ms and important to optimize
	- use current_time - start_time

- Cornercases: check what happens when pitch or roll in maximzed (tilt mcu all the way to left or right or down or up)
	- can be done with MCU, check motor values

- Define a test plan
	- Use different versions of the code in different branches to test right away
	- Test trimming functions for offset

- [Tomas] check USB interval value

- test p values (and change resolution e.g. p-value 1)
	- experiment with p-value higher resolution

## Lower priority

- fix state mismatch function
	- enter panic mode on state mismatch

- Big clean up 
	- remove unncessary comments
	- annotate all functions
	- add clear comments

- test logging and look at program like QT for storing data

# Questions to TA (can mail)

- What is the purpose of calibration mode?
- Should safe state always be able to be reached also without going through panic mode first?
- Should PC terminal show error when USB is disconnected?
- How to test drone response rate?
- How is logging tested?
