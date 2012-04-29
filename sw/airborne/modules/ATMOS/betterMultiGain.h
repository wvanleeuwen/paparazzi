#ifndef TRANSVEH_MULTIGAIN_H
#define TRANSVEH_MULTIGAIN_H

#include "std.h"

extern uint8_t gainSetId;
extern void transveh_multigain_init(void);
extern void transveh_multigain_periodic(void);
extern void SetGainSet(uint8_t gainSetId);

#endif  /* TRANSVEH_MULTIGAIN_H */
