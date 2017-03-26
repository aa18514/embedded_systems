# Readme.md

## recieving commands from the computer
"slre.h" library is used for parsing the command. Please   
enter the commands in the form R100 to set the number of revolutions.       
If a device needs to sent an instruction again, the USB cable needs to        
be plugged out and then reinserted again in the microchip.      

## new commands
A new command that does not conform to "R(-)?[0..9]\*" is ignored and        
the embedded system continues performing.      

## melody functionality
The melody function should not be used in conjunction with the movement      
command. Automatic tuning has not been implemented.      

## non-conformities
Commands for velocity need to be hardcorded in the main code as the regex      
for velocity has not been implemented. Please set the variable defined_velocity(line 77)   
in order to set the final velocity the motor should move at.    
