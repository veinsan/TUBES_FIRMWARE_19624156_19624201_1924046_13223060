#ifndef PID_H
#define PID_H

class PID {
private:
    float kp, ki, kd;
    float error, lastError, integral;

public:
    PID(float Kp, float Ki, float Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        error = 0;
        lastError = 0;
        integral = 0;
    }

    void compute(float setpoint, float current) {
        error = setpoint - current;
        integral += error;
        float derivative = error - lastError;

        float output = kp * error + ki * integral + kd * derivative;
        lastError = error;

        if (output > 2000) output = 2000;
        if (output < 1000) output = 1000;

        setOutput(output);
    }

    void setOutput(float output) {
        this->output = output;
    }

    float getOutput() {
        return this->output;
    }

    void tunePID(float Kp, float Ki, float Kd) {
        this->kp = Kp;
        this->ki = Ki;
        this->kd = Kd;
    }

private:
    float output;
};

#endif
