/*
 * DC_MOTOR.h
 *
 * Created: 5/12/2024 7:51:30 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#define MotorPort 'B'
#define MotorPin1 3
#define MotorPin2 4
void DC_motor_init(void);
void DC_motor_Forward(void);
void DC_motor_Backward(void);
void DC_motor_Stop(void);



#endif /* DC_MOTOR_H_ */