# esl

The manual mode functions is implemented with protocol. The allowed(keyboard) commands:\
  &emsp;&emsp;&emsp;&emsp;-lift up/down\
  &emsp;&emsp;&emsp;&emsp;-roll up/down\
  &emsp;&emsp;&emsp;&emsp;-yaw up/down\
  &emsp;&emsp;&emsp;&emsp;-pitch up/down\
  &emsp;&emsp;&emsp;&emsp;-mode switch among safe mode, manual mode, panic mode\
  \
But still have some problem,\
  &emsp;&emsp;&emsp;&emsp;-when the drone is in safe mode and we send a lift up command, which should be a manual mode command, the QR will say: 
  assertion "result == 1 && "QR: The mode in QR is not sync with PC or the action is not allowed in current mode!"" in terminal.\
  Because of this, the program will stop running, which I think we have to find a suitable mechanism to handle this.\
   \
  &emsp;&emsp;&emsp;&emsp;-I handle the 'ESC' command as same as a switch to panic mode. I think the 'ESC' should behave slightly different from switch to 
  painc mode in order to provide a way to exit the program.
   
