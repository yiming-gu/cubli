#ifndef CONTROL_H
#define CONTROL_H

#define MAX_VALUE 65535

typedef struct pid
{
    float err;
    float lastErr;
    float last2Err;
    float kp;
    float ki;
    float kd;
    float duty;
}PID;

typedef struct lqr
{
    float k1;
    float k2;
    float k3;
    float angle;
    float gyro;
    float lastGyro;
    float speed;
}LQR;


void pid_init(void);
float pid_control(float setSpeed,float actualSpeed);
void lqr_init(void);
float LQR_control(float angle, float gyro, float speed);
int read_encoder(void);
void encoder_manage(int *data);


#endif