#ifndef PID_H
#define PID_H

class PID
{
private:
    float kp, ki, kd;
    float error, lastError, integral;

public:
    PID(float Kp, float Ki, float Kd);
    void compute(float setpoint, float current);
    float getOutput();
    void tunePID(float Kp, float Ki, float Kd);
};

#endif
