#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_max;    // Anti-windup clamp on integral term
    float i;        // Accumulated integral
    float e_prev;   // Previous error (for derivative)
} pid_ctrl_t;

// Initialize PID state to zero with default anti-windup limit.
void pid_init(pid_ctrl_t* pid);

// Set PID gains. kp/ki/kd are the actual floating-point gains.
void pid_set_gains(pid_ctrl_t* pid, float kp, float ki, float kd);

// Set the anti-windup clamp on the integral term (symmetric: ±i_max).
void pid_set_imax(pid_ctrl_t* pid, float i_max);

// Compute one PID step.
// setpoint and measurement must be in the same units.
// dt is the time step in seconds.
// Returns the control output u.
float pid_update(pid_ctrl_t* pid, float setpoint, float measurement, float dt);

// Reset integral and previous error (call on mode change or fault clear).
void pid_reset(pid_ctrl_t* pid);

#endif // PID_H
