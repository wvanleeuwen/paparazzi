




#include "fbw_datalink.h"
#include "mcu_periph/uart.h"
#include "led.h"


#define __ModemLink(dev, _x) dev##_x
#define _ModemLink(dev, _x)  __ModemLink(dev, _x)
#define ModemLink(_x) _ModemLink(MODEM_LINK, _x)
#define ModemBuffer() ModemLink(ChAvailable())


#define __AutopilotLink(dev, _x) dev##_x
#define _AutopilotLink(dev, _x)  __AutopilotLink(dev, _x)
#define AutopilotLink(_x) _AutopilotLink(AUTOPILOT_LINK, _x)

#define AutopilotBuffer() AutopilotLink(ChAvailable())

static inline void autopilot_parse(char c)
{
  ModemLink(Transmit(c));
  RunOnceEvery(100,LED_TOGGLE(2));
}

static inline void modem_parse(char c)
{
  AutopilotLink(Transmit(c));
  RunOnceEvery(10,LED_TOGGLE(4));
}

#define ReadAutopilotBuffer() {                  \
    while (AutopilotLink(ChAvailable()))         \
      autopilot_parse(AutopilotLink(Getch()));      \
  }

#define ReadModemBuffer() {                  \
    while (ModemLink(ChAvailable()))         \
      modem_parse(ModemLink(Getch()));      \
  }



void FbwDataLinkEvent(void)
{
  ReadModemBuffer();
  ReadAutopilotBuffer();
}
