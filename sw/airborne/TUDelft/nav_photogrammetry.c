#include "nav_photogrammetry.h"
#include "estimator.h"

//static struct point survey_from;
//static struct point survey_to;

float photogrammetry_height;
float photogrammetry_overlap;
float photogrammetry_sidestep;
//float photogrammetry_wind_direction;
float photogrammetry_radius;
float photogrammetry_bb_offset;
uint8_t photo_lines_completed = 0;
uint8_t final_line = 0;
float wind_threshold;
//bool_t new_block;
float flight_lines_direction;
float vertical_sign;
//float flight_line_correction;
float from_step;
float to_step;
//float right_step;
float top_step;
bool_t top_from_below_to;
enum block_status block_status;
bool_t weak_wind;
//struct XYPoint pre_photogrammetry_position;

//block corners
float xmin;
float xmax;
float ymin;
float ymax;
float ytop_low;

#include "generated/flight_plan.h"
#include "generated/airframe.h"

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

// Calculate sine and cosine for rotation and Translation in x and y between World and Photogrammetry coordinates (usage depends on direction!!)
float T_sin;
float T_cos;
float TransX;
float TransY;

static void Rot_Trans_Init(float Zrot, uint8_t wp1)
{
  T_sin = sin(-Zrot); //rotation around DOWN axis in NED coordinates. (i.e. rotation photogrammetry -> world) CHECK uitleg
  T_cos = cos(-Zrot); //rotation around DOWN axis in NED coordinates. CHECK uitleg
  TransX = waypoints[wp1].x; //Positive in direction photogramm -> world CHECK uitleg
  TransY = waypoints[wp1].y; //Positive in direction photogramm -> world CHECK uitleg
}


static void TranslateAndRotateFromPhotogrammetrytoWorld(struct XYPoint *p, float transX, float transY)
{
	float temp;

	temp = p->x;
	p->x = p->x*T_cos+p->y*T_sin;
	p->y = temp*-T_sin+p->y*T_cos;

	p->x = p->x + transX;
	p->y = p->y + transY;
}

static void TranslateAndRotateFromWorldtoPhotogrammetry(struct XYPoint *p, float transX, float transY)
{
	float temp;

	p->x = p->x - transX;
	p->y = p->y - transY;

	temp = p->x;
	p->x = p->x*T_cos+p->y*-T_sin;
	p->y = temp*T_sin+p->y*T_cos;
}

bool_t  nav_photogrammetry_init(uint8_t wp1, float _photogrammetry_overlap, float photogrammetry_sidelap, float _wind_threshold, bool_t new_block)
{
  // Eerst 4 punten inlezen (Photo_to, Photo_from, Top_1 en Top_2)
  // Photogrammetry flightline direction is de baseline Photo_from --> Photo_to
  // Daarna, met sidelap zoals bepaald, vlieglijnen beide kanten op laten vliegen
  // De vlieglijnen worden aangepast aan de 'zijkanten' van het gebied, van photo_from/to naar de dichtstbijzijnde top
  // Als een van beide toppen verder van de baseline ligt dan de ander, wordt de lengte van de vliegpaden aangepast
  // Hiermee heb je vanaf grondstation invloed op vliegrichting en gebiedsgrens (handmatig afstemmen windrichting)!!

  // Define Photogrammetry Area and Altitude by 4 points
  struct XYPoint Base1;
  Base1.x = waypoints[wp1].x;
  Base1.y = waypoints[wp1].y;
  photogrammetry_height = waypoints[wp1].a;
  struct XYPoint Base2;
  Base2.x = waypoints[wp1+1].x;
  Base2.y = waypoints[wp1+1].y;
  struct XYPoint Top;
  Top.x = waypoints[wp1+2].x;
  Top.y = waypoints[wp1+2].y;
  struct XYPoint Top2;
  Top2.x = waypoints[wp1+3].x;
  Top2.y = waypoints[wp1+3].y;
  struct XYPoint Top_from;
  struct XYPoint Top_to;

   // Determine offset and rotation between photogrammetry and world coordinates
  flight_lines_direction = atan2(Base2.y-Base1.y,Base2.x-Base1.x);
  Rot_Trans_Init(flight_lines_direction, wp1);

  //Determine local coordinates for Base2, Top and Top2;
  TranslateAndRotateFromWorldtoPhotogrammetry(&Base2, TransX, TransY);
  TranslateAndRotateFromWorldtoPhotogrammetry(&Top, TransX, TransY);
  TranslateAndRotateFromWorldtoPhotogrammetry(&Top2, TransX, TransY);

  if(Top.x < Top2.x)
   {
    Top_from = Top;
    Top_to = Top2;
   }
  else
   {
    Top_from = Top2;
    Top_to = Top;
   }

  float minimum_top_distance = 300;

  if(fabs(Top_from.x - Top_to.x) < minimum_top_distance)
   {
    Top_from.x = Top_from.x - 0.5 * (minimum_top_distance -fabs(Top_from.x - Top_to.x));
    Top_to.x = Top_to.x + 0.5 * (minimum_top_distance -fabs(Top_from.x - Top_to.x));
   }

  if(Top.y < 0)
   {
     vertical_sign = -1.0;
   }
  else
   {
     vertical_sign = 1.0;
   }

  float viewing_ratio_height = ((float) PHOTOGRAMMETRY_SENSOR_HEIGHT) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  photogrammetry_sidestep = vertical_sign * viewing_ratio_height * photogrammetry_height * (1.0f - photogrammetry_sidelap);
  from_step  = Top_from.x/Top_from.y;
  to_step = (Top_to.x-Base2.x) / (Top_to.y-Base2.y);
  photogrammetry_radius =  vertical_sign * 50;
  photogrammetry_bb_offset = fabs(photogrammetry_radius) + 10;

  xmax = Base2.x;
  if(fabs(Top_from.y) > fabs(Top_to.y))
   {
    ymax = Top_from.y;
    ytop_low = Top_to.y;
    top_from_below_to = FALSE;
    if(fabs(Top_from.y - Top_to.y) < 2* photogrammetry_sidestep)
     top_step = 0;
    else
     top_step = (Top_to.x - (Top_from.x + minimum_top_distance)) / (Top_to.y - Top_from.y);
   }
  else
   {
    ymax = Top_to.y;
    ytop_low = Top_from.y;
    top_from_below_to = TRUE;
    if(fabs(Top_from.y - Top_to.y) < 2* photogrammetry_sidestep)
     top_step = 0;
    else
    top_step = ((Top_to.x - minimum_top_distance) - Top_from.x) / (Top_to.y - Top_from.y);
   }

  // Photogrammetry
  photogrammetry_overlap = _photogrammetry_overlap;
  wind_threshold = _wind_threshold;



  if(new_block)
   {
    photo_lines_completed = 0;
    block_status = ENTRY;
    final_line = FALSE;
   }
  else if(block_status == LINE_HEAD)
   block_status = ENTRY;
  else if(block_status == LINE_TAIL || block_status == RETURN)
   block_status = RETURN;
  else
   block_status = ENTRY;

  LINE_STOP_FUNCTION;
  return FALSE;
}

bool_t nav_photogrammetry(){

    struct XYPoint temp1;
    struct XYPoint temp2;
    struct XYPoint position_photogrammetry;

    switch(block_status){

    case ENTRY:
      temp1.y = photo_lines_completed * photogrammetry_sidestep + photogrammetry_radius;
      temp1.x = photogrammetry_bb_offset + temp1.y * from_step;
      temp2.y = (photo_lines_completed * photogrammetry_sidestep);
      temp2.x = photogrammetry_bb_offset + temp1.y * from_step;

      if(fabs(temp1.y) > fabs(ytop_low) && top_from_below_to == TRUE)
       {
        temp1.x = fabs(ytop_low) * from_step + fabs(temp1.y-ytop_low)*top_step + photogrammetry_bb_offset;
        temp2.x = fabs(ytop_low) * from_step + fabs(temp1.y-ytop_low)*top_step + photogrammetry_bb_offset;
       }
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, -photogrammetry_radius);
      if( fabs(estimator_x - temp2.x)<10 && fabs(estimator_y - temp2.y)<10)
       {
          block_status = LINE_HEAD;
          LINE_START_FUNCTION;
       }
      break;

    case LINE_HEAD:
      temp1.y = (photo_lines_completed * photogrammetry_sidestep);
      temp1.x = photogrammetry_bb_offset + temp1.y * from_step;
      if(fabs(temp1.y) > fabs(ytop_low) && top_from_below_to == TRUE)
       {
        temp1.x = fabs(ytop_low) * from_step + fabs(temp1.y-ytop_low)*top_step + photogrammetry_bb_offset;
       }
      temp2.y = (photo_lines_completed * photogrammetry_sidestep);
      temp2.x = xmax - photogrammetry_bb_offset + temp2.y * to_step;

      if(fabs(temp2.y) > fabs(ytop_low) && top_from_below_to == FALSE)
       {
        temp2.x = xmax + ytop_low * to_step - photogrammetry_bb_offset + fabs(temp2.y-ytop_low) * top_step;
       }
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;

      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
        TranslateAndRotateFromWorldtoPhotogrammetry(&temp2, TransX, TransY);
        LINE_STOP_FUNCTION;
        if((fabs(temp2.y) + 3 * fabs(photogrammetry_sidestep)) > fabs(ymax))
         {
          photo_lines_completed = photo_lines_completed + 2;
          final_line = TRUE;
          block_status = RETURN;
         }
        else if(photo_lines_completed == 0)
         {
          photo_lines_completed = photo_lines_completed + 3;
          block_status = RETURN;
         }
        else if(photo_lines_completed == 1)
         {
          photo_lines_completed = photo_lines_completed + 4;
          block_status = RETURN;
         }
        else
         {
          photo_lines_completed = photo_lines_completed + 5;
          block_status = RETURN;
         }

      }
      break;

    case RETURN:
      temp1.y = (photo_lines_completed * photogrammetry_sidestep)  - photogrammetry_radius;
      temp1.x = xmax - photogrammetry_bb_offset + temp1.y * to_step;
      temp2.y = (photo_lines_completed * photogrammetry_sidestep);
      temp2.x = xmax - photogrammetry_bb_offset + temp1.y * to_step;

      if((fabs(temp1.y) -fabs(photogrammetry_radius)) > fabs(ytop_low) && //7dec: bij fabs(temp1.y) - halve radius weggehaald
         top_from_below_to == FALSE)
       {
        temp1.x = xmax + ytop_low * to_step - photogrammetry_bb_offset + fabs(temp2.y-ytop_low) * top_step;
        temp2.x = temp1.x;
       }


      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, -(photogrammetry_radius));
      if( fabs(estimator_x - temp2.x) <10 && fabs(estimator_y - temp2.y) <10)
       {
         block_status = LINE_TAIL;

        LINE_START_FUNCTION;
       }
      break;

    case LINE_TAIL:
      temp1.y = (photo_lines_completed * photogrammetry_sidestep);
      temp2.y = temp1.y;
      temp1.x = xmax - photogrammetry_bb_offset + temp1.y * to_step;
      temp2.x = photogrammetry_bb_offset + (temp2.y - photogrammetry_radius) * from_step;

      if(fabs(temp1.y) > fabs(ytop_low) && top_from_below_to == FALSE)
       {
        temp1.x = xmax + ytop_low * to_step - photogrammetry_bb_offset + fabs(temp1.y-ytop_low) * top_step;
       }

      if(fabs(temp2.y) > fabs(ytop_low) && top_from_below_to == TRUE)
       {
        temp2.x = fabs(ytop_low) * from_step + fabs(temp1.y-ytop_low) * top_step + photogrammetry_bb_offset;
       }

      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
       LINE_STOP_FUNCTION;
       if(final_line == TRUE)
        {
         return FALSE;
        }
       else if(photo_lines_completed == 3)
        photo_lines_completed = photo_lines_completed -2;
       else
        photo_lines_completed = photo_lines_completed -3;

       block_status = ENTRY;
      }
      break;
    }

  NavVerticalAltitudeMode(photogrammetry_height, 0);
  NavVerticalAutoThrottleMode(0);
  return TRUE;
}

    /*case PRE_ENTRY: //Overbodig in huidige routine
       temp1.x = fabs(photogrammetry_radius);
       temp1.y = photogrammetry_radius;
       temp2.x = fabs(photogrammetry_radius);
       temp2.y = 0;
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

       nav_circle_XY(temp1.x, temp1.y, -photogrammetry_radius);
       if( fabs(estimator_x - temp2.x)<10 && fabs(estimator_y - temp2.y)<10)
        {
         LINE_START_FUNCTION;
         block_status = LINE_HEAD;
        }
       break; */

// ONDERSTAANDE ROUTINE WERKT! (nav_photogrammetry2 vereist 4 punten!)

/*
bool_t  nav_photogrammetry_init_extra(uint8_t wp1, uint8_t nr_of_wp, float _photogrammetry_overlap, float photogrammetry_sidelap,  float _wind_threshold, float block_margin, bool_t new_block)
{
  // Photogrammetry
  photogrammetry_height = waypoints[wp1].a;
  photogrammetry_overlap = _photogrammetry_overlap;
  wind_threshold = _wind_threshold;

  photogrammetry_wind_direction = atan2(-wind_east, -wind_north); //wind direction (where FROM, 0=North)
  Rot_Trans_Init(photogrammetry_wind_direction, wp1);
  int i;

  // Variables
  //float viewing_ratio_width = ((float) PHOTOGRAMMETRY_SENSOR_WIDTH) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  float viewing_ratio_height = ((float) PHOTOGRAMMETRY_SENSOR_HEIGHT) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  //float baseline = viewing_ratio_width * photogrammetry_height * (1.0f - photogrammetry_overlap);
  photogrammetry_sidestep = viewing_ratio_height * photogrammetry_height * (1.0f - photogrammetry_sidelap);

  xmin = 1e20;
  xmax = -1e20;
  ymin = 1e20;
  ymax = -1e20;

  // Determine Area of Interest (AoI) coordinates in Photogrammetry coordinates
  for(i=0;(i<nr_of_wp) ;i++) //&& ((wp1+i) < NB_WAYPOINT)
  {
    struct XYPoint temp;
    temp.x = waypoints[wp1+i].x;
    temp.y = waypoints[wp1+i].y;

    TranslateAndRotateFromWorldtoPhotogrammetry(&temp, TransX, TransY);
    if (temp.x < xmin)
      xmin = temp.x - block_margin;
    if (temp.x > xmax)
      xmax = temp.x + block_margin;
    if (temp.y < ymin)
      ymin = temp.y - block_margin;
    if (temp.y > ymax)
      ymax = temp.y + block_margin;
  }

  if( (ymax-ymin) < 100) //flight lines are at least 100 metres long
    ymax =  ymin + 100;

  if(new_block)
    photo_lines_completed = 0;

  //Determine whether wind is weak enough for to and from photo lines
  weak_wind = TRUE;
  if(wind_east * wind_east + wind_north * wind_north > wind_threshold * wind_threshold)
  {
    weak_wind = FALSE;
  }

  pre_photogrammetry_position.x = estimator_x;
  pre_photogrammetry_position.y = estimator_y;

  block_status = ENTRY;
  photogrammetry_radius =  50;

  //Determine photogrammetry coordinates for block corners
  for(i=0;(i<nr_of_wp) ;i++) //&& ((wp1+i) < NB_WAYPOINT)
  {
    struct XYPoint temp;
    temp.x = waypoints[wp1+i].x;
    temp.y = waypoints[wp1+i].y;

    TranslateAndRotateFromWorldtoPhotogrammetry(&temp, TransX, TransY);
    if(fabs(temp.x - (xmin + block_margin))<5)
    {
      left_corner.x = temp.x;
      left_corner.y = temp.y;
    }
    else if(fabs(temp.x -(xmax - block_margin))<5)
    {
      right_corner.x = temp.x;
      right_corner.y = temp.y;
    }
    else if(fabs(temp.y - (ymin + block_margin))<5)
    {
      bottom_corner.x = temp.x;
      bottom_corner.y = temp.y;
    }
    else if(fabs(temp.y - (ymax - block_margin))<5)
    {
      top_corner.x = temp.x;
      top_corner.y = temp.y;
    }
  }

 return FALSE;
}
*/

/*
// NAV_PHOTOGRAMMETRY_TRI_BB covers a rectangular area described by the bounding box around a triangle,
// defined by BASE_FROM, BASE_TO and TOP. Flight lines are parallel to and start from BASE_FROM to BASE_TO.
// The AC will not cross the line BASE_FROM to BASE_TO and as such this can be used as a limit (in case of highway, etc).

bool_t  nav_photogrammetry_init_tri_bb(uint8_t wp1, float _photogrammetry_overlap, float photogrammetry_sidelap, float _wind_threshold, bool_t new_block)
{
  // Eerst 3 punten inlezen (Base1, Base2 en Top)
  // Photogrammetry flightdirection is Base1 - Base2
  // Daarna, met sidelap zoals bepaald, vlieglijnen verleggen richting Top
  // Hoogte = hoogte van Base1
  // Hiermee heb je vanaf grondstation invloed op vliegrichting (handmatig afstemmen op omgeving&windrichting)!!

  // Define Photogrammetry Area and Altitude by 3 points
  struct XYPoint Base1;
  Base1.x = waypoints[wp1].x;
  Base1.y = waypoints[wp1].y;
  photogrammetry_height = waypoints[wp1].a;
  struct XYPoint Base2;
  Base2.x = waypoints[wp1+1].x;
  Base2.y = waypoints[wp1+1].y;
  struct XYPoint Top;
  Top.x = waypoints[wp1+2].x;
  Top.y = waypoints[wp1+2].y;

  flight_lines_direction = atan2(Base1.y-Base2.y,Base2.x-Base1.x);
  // Determine offset and rotation between photogrammetry and world coordinates
  Rot_Trans_Init(flight_lines_direction, wp1);

  //Determine local coordinates for Base2 and Top ==> gives limits for photogrammetry coordinates

  TranslateAndRotateFromWorldtoPhotogrammetry(&Base2, TransX, TransY);
  TranslateAndRotateFromWorldtoPhotogrammetry(&Top, TransX, TransY);

  float viewing_ratio_height = ((float) PHOTOGRAMMETRY_SENSOR_HEIGHT) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);

  if(Top.y < 0)
   {
     vertical_sign = -1.0;
     flight_line_correction = 0.0;
   }
  else
   {
     vertical_sign = 1.0;
     flight_line_correction = 180.0;
   }

  photogrammetry_sidestep = vertical_sign * viewing_ratio_height * photogrammetry_height * (1.0f - photogrammetry_sidelap);
  photogrammetry_radius =  vertical_sign * 50;

  xmin = 0;
  xmax = Base2.x;
  ymin = 0;
  ymax = Top.y;

  // Photogrammetry

  photogrammetry_overlap = _photogrammetry_overlap;
  wind_threshold = _wind_threshold;


  if(new_block)
    photo_lines_completed = 0;

  block_status = PRE_ENTRY;
  return FALSE;
}

bool_t nav_photogrammetry_tri_bb(){

    struct XYPoint temp1;
    struct XYPoint temp2;
    struct XYPoint position_photogrammetry;

    switch(block_status){

    case PRE_ENTRY:
       temp1.x = fabs(photogrammetry_radius);
       temp1.y = photogrammetry_radius;
       temp2.x = fabs(photogrammetry_radius);
       temp2.y = 0;
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

       nav_circle_XY(temp1.x, temp1.y, -photogrammetry_radius);
       if( nav_in_circle && NavQdrCloseTo(DegOfRad(flight_lines_direction)-flight_line_correction))
         block_status = LINE_HEAD;
       break;

    case ENTRY:
      temp1.x = fabs(photogrammetry_radius);
      temp1.y = photo_lines_completed * photogrammetry_sidestep + photogrammetry_radius;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, -photogrammetry_radius);
      if( nav_in_circle && NavQdrCloseTo(DegOfRad(flight_lines_direction)-flight_line_correction))
       {
          block_status = LINE_HEAD;
          LINE_START_FUNCTION;
       }
      break;

    case LINE_HEAD:
      temp1.x = fabs(photogrammetry_radius) +10;
      temp1.y = (photo_lines_completed * photogrammetry_sidestep);
      temp2.x = xmax - fabs(photogrammetry_radius) -10;
      temp2.y = (photo_lines_completed * photogrammetry_sidestep);
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;

      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
        if(fabs(position_photogrammetry.y) > fabs(ymax)-fabs(photogrammetry_sidestep))
         return FALSE;
        LINE_STOP_FUNCTION;
        photo_lines_completed++;
        block_status = RETURN;

      }
      break;

    case RETURN:
      temp1.x = xmax - fabs(photogrammetry_radius);
      temp1.y = (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;

      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, -photogrammetry_radius);
      if( nav_in_circle && NavQdrCloseTo(DegOfRad(flight_lines_direction + flight_line_correction - 180)))
       {
        block_status = LINE_TAIL;
        LINE_START_FUNCTION;
       }
      break;

    case LINE_TAIL:
      temp1.x = xmax - fabs(photogrammetry_radius) -10;
      temp1.y = (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;
      temp2.x = fabs(photogrammetry_radius) +10;
      temp2.y = (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;;
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;

      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);

      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
       if(fabs(position_photogrammetry.y) > fabs(ymax))
        return FALSE;
       block_status = ENTRY;
       LINE_STOP_FUNCTION;
      }
      break;

    }

  NavVerticalAltitudeMode(photogrammetry_height, 0);
  NavVerticalAutoThrottleMode(0);
  return TRUE;
}

nav_photogrammetry function: runs the photo acquisition sequence:
Consists of 4(+1) parts:
- PRE_ENTRY: fly to point 'below' photogrammetry block to guarantee correct entrance
- ENTRY: turn upwind to photo-line (switch to next stage when heading within 10 degrees, start taking pictures)
- LINE_HEAD: upwind line ( switch to next stage when fully completed, stop taking pictures ...
  turn to downwind in weak wind and start taking pictures, else switch to entry for next upwind line.
- RETURN: turn downwind for photo line (switch to next stage when heading within 10 degrees, start taking pictures )
- LINE_TAIL: downwind line (switch to next stage when fully completed, stop taking pictures)

nog opnemen: boundary lines om AoI weer te geven

bool_t nav_photogrammetry(){

    struct XYPoint temp1;
    struct XYPoint temp2;
    struct XYPoint position_photogrammetry;

    switch(block_status){

    case PRE_ENTRY:
       temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
       temp1.y = ymin - 2 * photogrammetry_radius;
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

       nav_route_xy(pre_photogrammetry_position.x, pre_photogrammetry_position.y, temp1.x, temp1.y);
       if( nav_approaching_xy(temp1.x, temp1.y, pre_photogrammetry_position.x, pre_photogrammetry_position.y, 0))
          block_status = ENTRY;
    break;

    case ENTRY:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
      temp1.y = ymin;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, photogrammetry_radius);
      if( NavQdrCloseTo(DegOfRad(photogrammetry_wind_direction)-180))
          block_status = LINE_HEAD;
      break;

    case LINE_HEAD:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep);
      temp1.y = ymin;
      temp2.x = xmin + (photo_lines_completed * photogrammetry_sidestep);
      temp2.y = ymax;
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);
      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);

      if(position_photogrammetry.y > ymin)
        LINE_START_FUNCTION;

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
        LINE_STOP_FUNCTION;
        photo_lines_completed++;
        if(weak_wind)
          block_status = RETURN;
        else
          block_status = ENTRY;
        //A block consists of a minimum of 3 flight lines and is completed if xmax is reached
        if((xmin + (photo_lines_completed * photogrammetry_sidestep) > xmax) && (photo_lines_completed >3))
          return FALSE;
      }
      break;

    case RETURN:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
      temp1.y = ymax;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, photogrammetry_radius);
      if( NavQdrCloseTo(DegOfRad(photogrammetry_wind_direction)))
        block_status = LINE_TAIL;
      break;

    case LINE_TAIL:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;
      temp1.y = ymax;
      temp2.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;
      temp2.y = ymin;
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);
      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);

      if(position_photogrammetry.y < ymax)
        LINE_START_FUNCTION;

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
          block_status = ENTRY;
          LINE_STOP_FUNCTION;
      }
      break;

    }

  NavVerticalAltitudeMode(photogrammetry_height, 0);
  NavVerticalAutoThrottleMode(0);
  return TRUE;
}

bool_t nav_photogrammetry2(){

    struct XYPoint temp1;
    struct XYPoint temp2;
    struct XYPoint position_photogrammetry;

    switch(block_status){

    case PRE_ENTRY:
       temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
       temp1.y = ymin - 2 * photogrammetry_radius;
       TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

       nav_route_xy(pre_photogrammetry_position.x, pre_photogrammetry_position.y, temp1.x, temp1.y);
       if( nav_approaching_xy(temp1.x, temp1.y, pre_photogrammetry_position.x, pre_photogrammetry_position.y, 0))
          block_status = ENTRY;
    break;

    case ENTRY:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
      if(temp1.x < bottom_corner.x)
        temp1.y = (ymin + ((bottom_corner.x - temp1.x) * (tan(-photogrammetry_wind_direction))));
      else
        temp1.y = (ymin + ((temp1.x - bottom_corner.x) / (tan(-photogrammetry_wind_direction))));
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, photogrammetry_radius);
      if( NavQdrCloseTo(DegOfRad(photogrammetry_wind_direction)-180))
          block_status = LINE_HEAD;
      break;

    case LINE_HEAD:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep);
      if(temp1.x < bottom_corner.x)
        temp1.y = (ymin + ((bottom_corner.x - temp1.x) * (tan(-photogrammetry_wind_direction))));
      else
        temp1.y = (ymin + ((temp1.x - bottom_corner.x) / (tan(-photogrammetry_wind_direction))));
      temp2.x = xmin + (photo_lines_completed * photogrammetry_sidestep);
      if(temp2.x < top_corner.x)
        temp2.y = (ymax - ((top_corner.x - temp2.x) / (tan(-photogrammetry_wind_direction))));
      else
        temp2.y = (ymax - ((temp2.x - top_corner.x) * (tan(-photogrammetry_wind_direction))));
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);
      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);

      if(position_photogrammetry.y > ymin)
        LINE_START_FUNCTION;

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
        LINE_STOP_FUNCTION;
        photo_lines_completed++;
        if(weak_wind)
          block_status = RETURN;
        else
          block_status = ENTRY;
        //A block consists of a minimum of 3 flight lines and is completed if xmax is reached
        if((xmin + (photo_lines_completed * photogrammetry_sidestep) > xmax) && (photo_lines_completed >3))
          return FALSE;
      }
      break;

    case RETURN:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + photogrammetry_radius;
      if(temp1.x < top_corner.x)
        temp1.y = (ymax - ((top_corner.x - temp1.x) / (tan(-photogrammetry_wind_direction))));
      else
        temp1.y = (ymax - ((temp1.x - top_corner.x) * (tan(-photogrammetry_wind_direction))));
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);

      nav_circle_XY(temp1.x, temp1.y, photogrammetry_radius);
      if( NavQdrCloseTo(DegOfRad(photogrammetry_wind_direction)))
        block_status = LINE_TAIL;
      break;

    case LINE_TAIL:
      temp1.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;
      if(temp1.x < top_corner.x)
        temp1.y = (ymax - ((top_corner.x - temp1.x) / (tan(-photogrammetry_wind_direction))));
      else
        temp1.y = (ymax - ((temp1.x - top_corner.x) * (tan(-photogrammetry_wind_direction))));
      temp2.x = xmin + (photo_lines_completed * photogrammetry_sidestep) + 2 * photogrammetry_radius;
      if(temp2.x < bottom_corner.x)
        temp2.y = (ymin + ((bottom_corner.x - temp2.x) * (tan(-photogrammetry_wind_direction))));
      else
        temp2.y = (ymin + ((temp2.x - bottom_corner.x) / (tan(-photogrammetry_wind_direction))));
      position_photogrammetry.x = estimator_x;
      position_photogrammetry.y = estimator_y;
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp1, TransX, TransY);
      TranslateAndRotateFromPhotogrammetrytoWorld(&temp2, TransX, TransY);
      TranslateAndRotateFromWorldtoPhotogrammetry(&position_photogrammetry, TransX, TransY);

      if(position_photogrammetry.y < ymax)
        LINE_START_FUNCTION;

      nav_route_xy(temp1.x, temp1.y, temp2.x, temp2.y);
      if(nav_approaching_xy(temp2.x, temp2.y, temp1.x, temp1.y, 0))
      {
          block_status = ENTRY;
          LINE_STOP_FUNCTION;
      }
      break;

    }

  NavVerticalAltitudeMode(photogrammetry_height, 0);
  NavVerticalAutoThrottleMode(0);
  return TRUE;
}
*/

/*
struct XYPoint left_corner;
struct XYPoint bottom_corner;
struct XYPoint top_corner;
struct XYPoint right_corner;

struct XYPoint BBox1;
struct XYPoint BBox2;
struct XYPoint BBox3;
struct XYPoint BBox4;

//AoI boundingbox corners
struct XYPoint bb_left_corner;
struct XYPoint bb_bottom_corner;
struct XYPoint bb_top_corner;
struct XYPoint bb_right_corner;
*/
