#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outMin, float outMax, float neutral)
        : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax), _neutral(neutral),
          _integral(0.0f), _prevError(0.0f), _lastOutput(neutral) {}

    void reset() {
        _integral = 0.0f;
        _prevError = 0.0f;
        _lastOutput = _neutral;
    }

    void setTunings(float kp, float ki, float kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    // Calcula a saída em microssegundos (us) para o ESC
    float compute(float target, float current, float dt_sec) {
        if (abs(target) < 0.5f && abs(current) < 2.0f) {
            reset();
            return _neutral;
        }

        float error = target - current;

        // Termo Integral com anti-windup
        _integral += error * dt_sec;
        float maxIntegral = 250.0f;
        if (_integral > maxIntegral) _integral = maxIntegral;
        else if (_integral < -maxIntegral) _integral = -maxIntegral;

        // Termo Derivativo
        float derivative = (dt_sec > 0.001f) ? (error - _prevError) / dt_sec : 0.0f;
        _prevError = error;

        // Feedforward elementar proporcional ao alvo (direção e offset base)
        float ff = 0.0f;
        if (target > 1.0f) {
            ff = 50.0f;
        } else if (target < -1.0f) {
            ff = -50.0f;
        }

        float output = _neutral + ff + (_kp * error) + (_ki * _integral) + (_kd * derivative);

        // Clamping aos limites seguros operacionais
        if (output > _outMax) output = _outMax;
        if (output < _outMin) output = _outMin;

        _lastOutput = output;
        return output;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _outMin;
    float _outMax;
    float _neutral;
    float _integral;
    float _prevError;
    float _lastOutput;
};

#endif // PID_CONTROLLER_H
