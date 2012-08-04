/** Automatic survey of a rectangle (defined by two points) (south-north or west-east sweep) */

#ifndef NAV_PHOTOGRAMMETRY_H
#define NAV_PHOTOGRAMMETRY_H

#include "subsystems/nav.h"

struct XYPoint {float x; float y;};

extern float photogrammetry_height;
extern float photogrammetry_overlap;
extern float photogrammetry_wind_direction;
extern uint8_t photo_lines_completed;

enum block_status {ENTRY, LINE_HEAD, RETURN, LINE_TAIL};

extern bool_t  nav_photogrammetry_init_extra(uint8_t wp1, uint8_t nr_of_wp, float _photogrammetry_overlap, float _photogrammetry_sidelap, float _wind_threshold, float block_margin, bool_t new_block);
extern bool_t  nav_photogrammetry_init(uint8_t wp1, float _photogrammetry_overlap, float _photogrammetry_sidelap, float _wind_threshold, bool_t new_block);
extern bool_t  nav_photogrammetry(void);
//extern bool_t  nav_photogrammetry_tri_bb(void);

#define PhotogrammetryInitDefault(_wp1, _nr_of_wp) nav_photogrammetry_init( _wp1, _nr_of_wp, 0.8f, 0.8f, 70.0f, 35.0f);
#define PhotogrammetryInit_extra(_wp1, _nr_of_wp, _overlap, _sidelap, _wind_threshold, _block_margin, _new_block) nav_photogrammetry_init_extra(_wp1, _nr_of_wp, _overlap, _sidelap, _wind_threshold, _block_margin, _new_block)
#define PhotogrammetryInit(_wp1, _overlap, _sidelap, _wind_threshold, _new_block) nav_photogrammetry_init(_wp1, _overlap, _sidelap, _wind_threshold, _new_block)

#define Photogrammetry() nav_photogrammetry()
//#define Photogrammetry_tri_bb() nav_photogrammetry_tri_bb()
#endif
