#ifndef TRANSVEH_HOVER_PROPS_OFF_H
#define TRANSVEH_HOVER_PROPS_OFF_H

#include "std.h"


extern void transveh_hover_props_off_init(void);
extern void transveh_hover_props_off_periodic(void);
extern void transveh_hover_props_off_set(bool_t stop);

extern void HoverPropsOn(void);
extern void HoverPropsOff(void);

extern bool_t killHoverProps;

#endif  /* TRANSVEH_HOVER_PROPS_OFF */
