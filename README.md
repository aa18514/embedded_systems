# embedded_systems

slre.h library is used for parsing the commands sent via the serial port.
commands are sent in the form R123 to set the number of revolutions.
Regex for velocity has not been implement but velocity can be changed by changing the variable defined_velocity
Motor should be disconnected and connected again if a new command is sent in.
