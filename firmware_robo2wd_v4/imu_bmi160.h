#ifndef IMU_BMI160_H
#define IMU_BMI160_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// =========================================================================
// DRIVER OTIMIZADO PARA IMU BOSCH BMI160 VIA I2C NATIVO
// - Inicialização do acelerômetro e giroscópio no modo Normal (100 Hz).
// - Calibração de offset estático do giroscópio (compensação de drift).
// - Integração numérica de orientação angular Yaw (eixo Z).
// =========================================================================

#define BMI160_DEFAULT_ADDR 0x68
#define BMI160_CHIP_ID_REG  0x00
#define BMI160_DATA_REG     0x0C // Início dos dados: Gyr X (0x0C) até Acc Z (0x17)
#define BMI160_CMD_REG      0x7E

class IMU_BMI160 {
public:
    IMU_BMI160() : _addr(BMI160_DEFAULT_ADDR), _initialized(false),
                   _gzOffset(0.0f), _yawAngleDeg(0.0f), _currentYawRateDps(0.0f) {}

    bool begin(uint8_t addr = BMI160_DEFAULT_ADDR) {
        _addr = addr;
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000); // 400 kHz Fast Mode

        // 1. Verificar Chip ID (deve ser 0xD1)
        uint8_t chip_id = readReg(_addr, BMI160_CHIP_ID_REG);
        if (chip_id != 0xD1) {
            // Tenta endereço alternativo 0x69 caso 0x68 falhe
            if (chip_id == 0xFF || chip_id == 0x00) {
                _addr = 0x69;
                chip_id = readReg(_addr, BMI160_CHIP_ID_REG);
            }
            if (chip_id != 0xD1) {
                return false;
            }
        }

        // 2. Soft-reset no sensor
        writeReg(_addr, BMI160_CMD_REG, 0xB6);
        delay(80);

        // 3. Ligar Acelerômetro (CMD: 0x11) e Giroscópio (CMD: 0x15) em modo Normal
        writeReg(_addr, BMI160_CMD_REG, 0x11);
        delay(50);
        writeReg(_addr, BMI160_CMD_REG, 0x15);
        delay(80);

        // 4. Configurar range do Giroscópio para +-250 deg/s (131.2 LSB/dps)
        writeReg(_addr, 0x43, 0x03);
        // 5. Configurar range do Acelerômetro para +-2G (16384 LSB/g)
        writeReg(_addr, 0x41, 0x03);

        // 6. Configurar Output Data Rate para 100 Hz em ambos
        writeReg(_addr, 0x42, 0x28); // Gyr ODR 100Hz
        writeReg(_addr, 0x40, 0x28); // Acc ODR 100Hz
        delay(20);

        _initialized = true;
        resetYaw();
        return true;
    }

    // Calibra o offset do giroscópio no eixo Z enquanto o robô está parado no boot
    void calibrateGyro(uint16_t samples = 150) {
        if (!_initialized) return;
        float sum = 0.0f;
        for (uint16_t i = 0; i < samples; i++) {
            // Leitura direta do eixo Z para que giro a DIREITA (Horario) gere valor POSITIVO (+)
            float gz = readRawGyroZ();
            sum += gz;
            delay(5);
        }
        _gzOffset = sum / (float)samples;
    }

    // Atualiza a integração do ângulo de Yaw (chamar periodicamente a cada 20 ms)
    void update(float dt_sec) {
        if (!_initialized) return;

        // Convencao: giro para a DIREITA (horario) = positivo (+)
        float rawGz = readRawGyroZ();
        _currentYawRateDps = rawGz - _gzOffset;

        // Zona morta de ruído (evita drift quando parado)
        if (abs(_currentYawRateDps) < 0.25f) {
            _currentYawRateDps = 0.0f;
        }

        _yawAngleDeg += _currentYawRateDps * dt_sec;
    }

    void resetYaw() {
        _yawAngleDeg = 0.0f;
        _currentYawRateDps = 0.0f;
    }

    float getYawAngleDeg() const { return _yawAngleDeg; }
    float getYawRateDps() const { return _currentYawRateDps; }
    bool isInitialized() const { return _initialized; }

private:
    uint8_t readReg(uint8_t addr, uint8_t reg) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)addr, (uint8_t)1);
        if (Wire.available()) {
            return Wire.read();
        }
        return 0xFF;
    }

    void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    float readRawGyroZ() {
        Wire.beginTransmission(_addr);
        Wire.write(0x10); // Registrador GYR_Z_LSB
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)_addr, (uint8_t)2) == 2) {
            int16_t rawZ = Wire.read() | (Wire.read() << 8);
            return (float)rawZ / 131.2f; // Escala +-250 deg/s
        }
        return 0.0f;
    }

    uint8_t _addr;
    bool    _initialized;
    float   _gzOffset;
    float   _yawAngleDeg;
    float   _currentYawRateDps;
};

#endif // IMU_BMI160_H
