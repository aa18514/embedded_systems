# embedded_systems
slre.h library is used for parsing the commands sent via the serial port. 
commands are sent in the form which are either R500 but not R234V56.
commands are sent once at a time for setting the number of rotations, velocity needs to be hardcorded. 
commands for playing the tune are not set.
Motor should be disconnected and connected again if a new command is sent in.
