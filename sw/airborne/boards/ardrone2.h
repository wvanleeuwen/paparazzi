#ifndef CONFIG_ARDRONE2
#define CONFIG_ARDRONE2

#define BOARD_ARDRONE2

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* Internal communication */
#define ARDRONE_NAVDATA_PORT 5554
#define ARDRONE_AT_PORT 5556
#define ARDRONE_NAVDATA_BUFFER_SIZE 2048
#define ARDRONE_IP "192.168.1.1"

#endif /* CONFIG_ARDRONE2 */
