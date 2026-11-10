#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

class MotorControl {
public:
    MotorControl() : _pinM1(PIN_ESC_CH1), _pinM2(PIN_ESC_CH2) {}

    void begin() {
        // ESP32 Arduino Core 3.x suporta ledcAttach(pin, freq, resolution)
        ledcAttach(_pinM1, ESC_PWM_FREQ, ESC_PWM_RES_BITS);
        ledcAttach(_pinM2, ESC_PWM_FREQ, ESC_PWM_RES_BITS);

        // Envia sinal neutro imediatamente
        setMicrosecondsM1(ESC_NEUTRO_US);
        setMicrosecondsM2(ESC_NEUTRO_US);
    }

    void setMicrosecondsM1(uint32_t us) {
        if (us < ESC_MIN_US) us = ESC_MIN_US;
        if (us > ESC_MAX_US) us = ESC_MAX_US;
        uint32_t duty = usToDuty(us);
        ledcWrite(_pinM1, duty);
    }

    void setMicrosecondsM2(uint32_t us) {
        if (us < ESC_MIN_US) us = ESC_MIN_US;
        if (us > ESC_MAX_US) us = ESC_MAX_US;
        uint32_t duty = usToDuty(us);
        ledcWrite(_pinM2, duty);
    }

    void stopAll() {
        setMicrosecondsM1(ESC_NEUTRO_US);
        setMicrosecondsM2(ESC_NEUTRO_US);
    }

private:
    uint8_t _pinM1;
    uint8_t _pinM2;

    // Período de 50Hz = 20.000 microssegundos. Resolução de 16 bits = 65535.
    inline uint32_t usToDuty(uint32_t us) {
        return (uint32_t)((((uint64_t)us) * 65535ULL) / 20000ULL);
    }
};

#endif // MOTOR_CONTROL_H
