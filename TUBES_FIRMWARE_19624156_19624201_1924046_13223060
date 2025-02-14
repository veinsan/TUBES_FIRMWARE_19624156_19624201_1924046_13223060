#include <Servo.h>
#include <HCSR04.h>
#include "PID.h"

#define TRIG_PIN 5
#define ECHO_PIN 6
#define ESC_PIN 9

float setpoint = 30.0;

PID pid(1.0, 0.1, 0.01);

HCSR04 ultrasonic(TRIG_PIN, ECHO_PIN);

Servo esc;

void setup() {
    Serial.begin(9600);
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(1000);
    delay(2000);

    if (Serial.available() > 0) {
        float Kp = Serial.parseFloat();
        float Ki = Serial.parseFloat();
        float Kd = Serial.parseFloat();
        pid.tunePID(Kp, Ki, Kd);
    }
}

void loop() {
    long distance = ultrasonic.dist();

    float output = pid.getOutput();

    int motor_speed = constrain(output, 1000, 2000);
    esc.writeMicroseconds(motor_speed);

    delay(100);
}
