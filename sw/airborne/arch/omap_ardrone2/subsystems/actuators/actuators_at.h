

#ifndef BOARDS_ARDRONE_ACTUATORS_H
#define BOARDS_ARDRONE_ACTUATORS_H

#include <stdio.h>

extern void actuators_init(void);
#define SetActuatorsFromCommands(command) { printf("Commads: %d %d %d %d\n", command[0], command[1], command[2], command[3]); }

#endif /* BOARDS_ARDRONE_ACTUATORS_H */
