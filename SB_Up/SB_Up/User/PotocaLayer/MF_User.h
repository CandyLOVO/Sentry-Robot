#ifndef MF_USER_H
#define MF_USER_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "Motor.h"
#include "Can_user.h"

void Start_MF_send(int16_t ID);
void Current_Control_MF_send(int16_t ID,int16_t iqControl);
int16_t Current_Limit_MF(int16_t current);
#ifdef __cplusplus
}
#endif

#endif