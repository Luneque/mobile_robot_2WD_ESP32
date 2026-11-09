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

    float compute(float target, float current, float dt_sec) {
        if (abs(target) < 0.5f && abs(current) < 2.0f) {
            reset();
            return _neutral;
        }

        float error = target - current;

        // Anti-windup
        _integral += error * dt_sec;
        const float maxIntegral = 250.0f;
        if (_integral > maxIntegral) _integral = maxIntegral;
        else if (_integral < -maxIntegral) _integral = -maxIntegral;

        float derivative = (dt_sec > 0.001f) ? (error - _prevError) / dt_sec : 0.0f;
        _prevError = error;

        float ff = 0.0f;
        if (target > 1.0f) {
            ff = ESC_DEADBAND_US;
        } else if (target < -1.0f) {
            ff = -ESC_DEADBAND_US;
        }

        float output = _neutral + ff + (_kp * error) + (_ki * _integral) + (_kd * derivative);

        if (output > _outMax) output = _outMax;
        if (output < _outMin) output = _outMin;

        _lastOutput = output;
        return output;
    }

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax, _neutral;
    float _integral, _prevError, _lastOutput;
};

#endif // PID_CONTROLLER_H
