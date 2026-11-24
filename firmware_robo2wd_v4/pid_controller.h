#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// =========================================================================
// CURVA PWM x RPM (feedforward por velocidade)
// Medida em bancada pelo utilitario diag_motor_curve (robo suspenso), com a
// media dos dois motores. A curva inclui a zona morta (1500-1600 us).
// O PID usa essa curva para estimar o pulso necessario a velocidade alvo,
// corrigida pelo fator de carga (CURVA_FATOR_CARGA) do robo no solo.
//
// Medicao de 2026-11-16 (diag_motor_curve, robo suspenso):
//   PWM_us: 1500  1550  1600  1650  1700  1750  1800  1850
//   RPM esq:  0.0   0.0   3.9  30.2  60.0  81.2  98.7 111.9
//   RPM dir:  0.0   0.0   0.0  39.8  70.1  92.1 107.1 119.2
//   media:     0.0   0.0   1.9  35.0  65.0  86.7 102.9 115.6
// Validada no solo: com fator 0.80, 55 RPM alvo -> ~1719 us -> ~55 RPM real.
// =========================================================================
static const float CURVA_RPM[] = {0.0f,  1.9f,  35.0f, 65.0f,  86.7f, 102.9f, 115.6f};
static const float CURVA_PWM[] = {1600.0f, 1600.0f, 1650.0f, 1700.0f, 1750.0f, 1800.0f, 1850.0f};
static const int   CURVA_PONTOS = 7;

// Interpola linearmente a curva: dado um RPM (sem carga), retorna o PWM (us).
static float curvaPWM_paraRPM(float rpm) {
    if (rpm <= CURVA_RPM[0]) return CURVA_PWM[0];
    if (rpm >= CURVA_RPM[CURVA_PONTOS - 1]) return CURVA_PWM[CURVA_PONTOS - 1];
    for (int i = 0; i < CURVA_PONTOS - 1; i++) {
        if (rpm >= CURVA_RPM[i] && rpm <= CURVA_RPM[i + 1]) {
            float f = (rpm - CURVA_RPM[i]) / (CURVA_RPM[i + 1] - CURVA_RPM[i]);
            return CURVA_PWM[i] + f * (CURVA_PWM[i + 1] - CURVA_PWM[i]);
        }
    }
    return CURVA_PWM[CURVA_PONTOS - 1];
}

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

        // Anti-windup com saturação dinâmica
        _integral += error * dt_sec;
        const float maxIntegral = 300.0f;   // 200 -> 300: margem para o novo Ki atingir o alvo de 55 RPM sob carga
        if (_integral > maxIntegral) _integral = maxIntegral;
        else if (_integral < -maxIntegral) _integral = -maxIntegral;

        float derivative = (dt_sec > 0.001f) ? (error - _prevError) / dt_sec : 0.0f;
        _prevError = error;

        // Feedforward por velocidade: interpola a curva PWM x RPM medida em bancada,
        // corrigida pelo fator de carga do robo no solo. O PID passa a corrigir so o erro.
        float ff = 0.0f;
        if (target > 1.0f) {
            ff = curvaPWM_paraRPM(target / CURVA_FATOR_CARGA) - _neutral;
        } else if (target < -1.0f) {
            ff = -(curvaPWM_paraRPM(-target / CURVA_FATOR_CARGA) - _neutral);
        }

        float output = _neutral + ff + (_kp * error) + (_ki * _integral) + (_kd * derivative);

        if (output > _outMax) output = _outMax;
        if (output < _outMin) output = _outMin;

        _lastOutput = output;
        return output;
    }

    float getLastOutput() const { return _lastOutput; }

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
