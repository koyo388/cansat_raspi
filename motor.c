#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>
#include "motor.h"

static const int LEFT_MOTOR1 = 23;//GPIO23
static const int LEFT_MOTOR2 = 24;//GPIO24
static const int RIGHT_MOTOR1 = 27;//GPIO27
static const int RIGHT_MOTOR2 = 22;//GPIO17
static const int PWM_RANGE = 255;
static const int INITIAL_PWM_VAL = 0;
static const int ZERO_PWM_VAL = 0;
//static const int MAX_PWM_VAL = 100;

int pwm_initialize()
{
	//wiring Pi initialize
	if (wiringPiSetupGpio() != 0)
	{
		printf("motor_setup_failed\n");
	}
	pwmSetMode(PWM_MODE_MS);
	//soft pwm initialize
	softPwmCreate(RIGHT_MOTOR1, INITIAL_PWM_VAL, PWM_RANGE);
	softPwmCreate(RIGHT_MOTOR2, INITIAL_PWM_VAL, PWM_RANGE);
	softPwmCreate(LEFT_MOTOR1, INITIAL_PWM_VAL, PWM_RANGE);
	softPwmCreate(LEFT_MOTOR2, INITIAL_PWM_VAL, PWM_RANGE);
	return 0;
}

/*
pwm_value‚Í0~100‚Ì’l‚ð‚Æ‚éB
“à•”‚Åthreading‚µ‚Ä‚é‚±‚Æ‚É’ˆÓ
*/

void motor_control(int L, int R)
{

	if (L >= 0 && R >= 0) {
		softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
		softPwmWrite(RIGHT_MOTOR1, R);
		softPwmWrite(LEFT_MOTOR1, L);
		softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	}
	else if (L == 0 && R == 0) {
		softPwmWrite(RIGHT_MOTOR1, ZERO_PWM_VAL);
		softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
		softPwmWrite(LEFT_MOTOR1, ZERO_PWM_VAL);
		softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	}
	else if (R >= 0 && L <= 0) {
		softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
		softPwmWrite(RIGHT_MOTOR1, R);
		softPwmWrite(LEFT_MOTOR1, -1*(L));
		softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	}
	else if (L >= 0 && R <= 0) {
		softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
		softPwmWrite(RIGHT_MOTOR1, -1*(R));
		softPwmWrite(LEFT_MOTOR1, L);
		softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	}

}

int motor_stop()
{
	printf("MOTOR stop\n");
	softPwmWrite(RIGHT_MOTOR1, ZERO_PWM_VAL);
	softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
	softPwmWrite(LEFT_MOTOR1, ZERO_PWM_VAL);
	softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	return 0;
}

int motor_forward(int pwm_value)
{
	printf("MOTOR forward\n");
	softPwmWrite(RIGHT_MOTOR1, pwm_value);
	softPwmWrite(RIGHT_MOTOR2, ZERO_PWM_VAL);
	softPwmWrite(LEFT_MOTOR1, pwm_value);
	softPwmWrite(LEFT_MOTOR2, ZERO_PWM_VAL);
	return 0;
}

int motor_back(int pwm_value)
  {   
       printf("MOTOR back\n");
        softPwmWrite(RIGHT_MOTOR1,ZERO_PWM_VAL);
         softPwmWrite(RIGHT_MOTOR2,pwm_value);
	      softPwmWrite(LEFT_MOTOR1,ZERO_PWM_VAL);
	       softPwmWrite(LEFT_MOTOR2,pwm_value);
	        return 0;
	     }
		  
		   int motor_right(int pwm_value)
		    {
		         printf("MOTOR right\n");
			      softPwmWrite(RIGHT_MOTOR1,ZERO_PWM_VAL);
			       softPwmWrite(RIGHT_MOTOR2,pwm_value);
			        softPwmWrite(LEFT_MOTOR1,pwm_value);
			         softPwmWrite(LEFT_MOTOR2,ZERO_PWM_VAL);
				      return 0;
				   }

int motor_left(int pwm_value)
  {   
       printf("MOTOR left\n");
        softPwmWrite(RIGHT_MOTOR1,pwm_value); 
		softPwmWrite(RIGHT_MOTOR2,ZERO_PWM_VAL); 
         softPwmWrite(LEFT_MOTOR1,ZERO_PWM_VAL);  
	      softPwmWrite(LEFT_MOTOR2,pwm_value);
	       return 0;
	    }
