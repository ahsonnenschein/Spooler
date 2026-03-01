#include "pid.h"

void pid_init(pid_ctrl_t* pid)
{
    pid->kp     = 0.0f;
    pid->ki     = 0.0f;
    pid->kd     = 0.0f;
    pid->i_max  = 10000.0f;  // Conservative default; tune via Modbus
    pid->i      = 0.0f;
    pid->e_prev = 0.0f;
}

void pid_set_gains(pid_ctrl_t* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_imax(pid_ctrl_t* pid, float i_max)
{
    pid->i_max = i_max;
}

float pid_update(pid_ctrl_t* pid, float setpoint, float measurement, float dt)
{
    float e = setpoint - measurement;

    // Proportional
    float p_term = pid->kp * e;

    // Integral with anti-windup clamp
    pid->i += pid->ki * e * dt;
    if (pid->i >  pid->i_max) pid->i =  pid->i_max;
    if (pid->i < -pid->i_max) pid->i = -pid->i_max;

    // Derivative (on error)
    float d_term = (dt > 0.0f) ? (pid->kd * (e - pid->e_prev) / dt) : 0.0f;
    pid->e_prev = e;

    return p_term + pid->i + d_term;
}

void pid_reset(pid_ctrl_t* pid)
{
    pid->i      = 0.0f;
    pid->e_prev = 0.0f;
}
