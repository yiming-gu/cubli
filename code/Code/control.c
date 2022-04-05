#include "control.h"
#include "sys.h"

PID pid;
LQR lqr;

void pid_init(void)
{
    pid.err = 0;
    pid.lastErr = 0;
    pid.last2Err = 0;
    pid.kp = 8;
    pid.ki = 0.8;
    pid.kd = 7;
    pid.duty = 0;
}

//LQR系数初始化
void lqr_init(void)
{
    lqr.k1 = 100;//100
    lqr.k2 = 10;//10
    lqr.k3 = 1;//1
    lqr.speed = 0;
}

float pid_control(float setSpeed,float actualSpeed)//增量式PID
{
    pid.err = setSpeed - actualSpeed;
    pid.duty += pid.kp * (pid.err - pid.lastErr) + pid.ki * pid.err - pid.kd * (pid.err - 2 * pid.lastErr + pid.last2Err);
    pid.lastErr = pid.err;
    pid.last2Err = pid.lastErr;
    if(pid.duty >  1800)   pid.duty =  1800;
    if(pid.duty < -1800)   pid.duty = -1800;
    return pid.duty;
}

float LQR_control(float angle, float gyro, float speed)
{
    lqr.speed = lqr.k1*angle + lqr.k2*gyro + lqr.k3*speed;
    return lqr.speed;
}


//编码器数据读取
int read_encoder(void)
{
    int encoder_val;
    encoder_val = (int)TIM2->CNT;
    TIM2->CNT = 0;
    return encoder_val;
}


void encoder_manage(int *data)
{
    //编码器反向时，转为与正向时对应的负数
    if(*data>(MAX_VALUE*0.5f))
        *data = *data - MAX_VALUE;
    else
        *data = *data;
}