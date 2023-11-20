#ifndef __MOTION_OVERLAY_H__
#define __MOTION_OVERLAY_H__
#include "main.h"

void translate_3508(int16_t x,int16_t y);
void translate_6020(int16_t x,int16_t y);
void compound_movement_3508(int16_t vx,int16_t vy);
void compound_movement_6020(int16_t vx,int16_t vy);
//float compound_movement_6020(int16_t vx,int16_t vy,int n);
void rotate_6020();

#endif