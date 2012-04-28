#ifndef TEMP_ATMEGA48_H
#define TEMP_ATMEGA48_H

#include "std.h"

#ifndef ATMEGA_TX_SIZE
#define ATMEGA_TX_SIZE	12
#endif

#ifndef ATMEGA_RX_SIZE
#define ATMEGA_RX_SIZE	12
#endif

extern uint8_t to_atmega48[ATMEGA_TX_SIZE];
extern uint8_t from_atmega48[ATMEGA_RX_SIZE];

void atmega48_init(void);
void atmega48_periodic(void);
void atmega48_event(void);

#define ParsePayloadCommand() { 				\
  {								\
    to_atmega48[0] = DL_PAYLOAD_COMMAND_command(dl_buffer)[0];	\
    to_atmega48[1] = DL_PAYLOAD_COMMAND_command(dl_buffer)[1];	\
  } 								\
}



#endif
