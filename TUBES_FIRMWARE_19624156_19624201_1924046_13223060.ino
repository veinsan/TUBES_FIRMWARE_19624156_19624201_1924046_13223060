#include <Servo.h>
#include <HCSR04.h>

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

#define TRIG_PIN 5
#define ECHO_PIN 6
#define ESC_PIN 9

float setpoint = 30.0;
PID pid(1.0, 0.1, 0.01);

Servo esc;
HCSR04 ultrasonic(TRIG_PIN, ECHO_PIN);

class Motor {
private:
    Servo esc;
public:
    void init() {
        esc.attach(ESC_PIN);
        esc.writeMicroseconds(1500);
    }
    void setSpeed(int speed) {
        esc.writeMicroseconds(speed);
    }
};

Motor motor;

long duration;
float distance;

void setup() {
    Serial.begin(115200);
    motor.init();
}

void loop() {
    distance = ultrasonic.dist();

    Serial.print("Distance: ");
    Serial.println(distance);

    int motorSpeed = 1500;

    if (distance >= 0 && distance <= 100) {
        motorSpeed = 1000;  
    }
    else if (distance > 100 && distance <= 250) {
        motorSpeed = map(distance, 100, 250, 1000, 1500);
    }
    else if (distance > 250 && distance <= 336) {
        motorSpeed = map(distance, 250, 336, 1500, 2000);
    }

    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
    motor.setSpeed(motorSpeed);

    delay(100);
}