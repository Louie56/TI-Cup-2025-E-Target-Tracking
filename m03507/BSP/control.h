#ifndef __CONTROL_H
#define __CONTROL_H

#include "board.h"
#include "math.h"

void stepper_x_run(int tim,float step,float subdivide,int8_t dir);
void stepper_y_run(int tim, float step, float subdivide, int8_t dir);
void turn_coordinate(float x, float y);
void drawline(float X0, float Y0, float Xe, float Ye);
void drawcircle(float x0, float y0, float R, uint8_t SorN);
#endif
