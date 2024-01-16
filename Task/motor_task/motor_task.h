#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "can_feedback.h"

void system_init(void);
void CHASSIS_TASK(void);
void chassis_mode(CHASSIS *_chassis);
void chassis_rc_slove(CHASSIS *_chassis);
void chassis_behavier_get(CHASSIS *_chassis);



#endif
