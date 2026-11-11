/*
 * calib_imu_polaridade.ino
 * Validador de Polaridade e Orientacao dos Eixos da IMU BMI160
 *
 * Objetivo:
 *  Verificar se o giro fisico do robo no sentido HORARIO (Curva a Direita)
 *  produz sinal POSITIVO (+) ou NEGATIVO (-) no eixo Z do giroscopio.
 *
 * Taxa da Serial: 115200 baud
 * Conexoes: SDA=GPIO 21, SCL=GPIO 22
 */

#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define BMI160_ADDR         0x68
#define BMI160_CHIP_ID_REG  0x00
#define BMI160_CMD_REG      0x7E

float g_gzOffset = 0.0f;
float g_yawAngleDeg = 0.0f;
unsigned long g_lastTime = 0;

uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

float readRawGyroZ() {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(0x10); // GYR_Z_LSB
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)2) == 2) {
        int16_t rawZ = Wire.read() | (Wire.read() << 8);
        return (float)rawZ / 131.2f; // Escala +-250 deg/s
    }
    return 0.0f;
}

void calibrarOffsetParado() {
    Serial.println("\n[CALIB] Calibrando offset estatico do Giroscopio...");
    Serial.println(">>> ATENCAO: MANTENHA O ROBO TOTALMENTE PARADO NO CHAO! <<<");
    delay(1000);

    float sum = 0.0f;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        sum += readRawGyroZ();
        delay(5);
    }
    g_gzOffset = sum / (float)N;
    g_yawAngleDeg = 0.0f;

    Serial.printf("[OK] Offset calculado: %+.3f deg/s\n", g_gzOffset);
    Serial.println("=========================================================");
    Serial.println(" INSTRUCAO DO TESTE DE POLARIDADE:");
    Serial.println("  1. Gire o robo manualmente 90 GRAUS PARA A DIREITA (Sentido Horario).");
    Serial.println("  2. Observe o valor de 'Yaw Integrado' na serial.");
    Serial.println("  Envie 'z' pelo monitor para zerar o angulo a qualquer momento.");
    Serial.println("=========================================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==============================================");
    Serial.println("    TESTE DE POLARIDADE - IMU BOSCH BMI160    ");
    Serial.println("==============================================");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    uint8_t chip_id = readReg(BMI160_CHIP_ID_REG);
    Serial.printf("[BMI160] Chip ID: 0x%02X\n", chip_id);

    // Soft reset
    writeReg(BMI160_CMD_REG, 0xB6);
    delay(80);

    // Liga Giroscópio modo Normal
    writeReg(BMI160_CMD_REG, 0x15);
    delay(80);

    // Range +-250 deg/s e ODR 100Hz
    writeReg(0x43, 0x03);
    writeReg(0x42, 0x28);
    delay(20);

    calibrarOffsetParado();
    g_lastTime = millis();
}

unsigned long lastPrint = 0;

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z' || c == 'r' || c == 'Z' || c == 'R') {
            g_yawAngleDeg = 0.0f;
            Serial.println("\n>>> ANGULO YAW ZERADO! <<<\n");
        }
    }

    unsigned long now = millis();
    float dt = (now - g_lastTime) / 1000.0f;
    g_lastTime = now;

    float rawGz = readRawGyroZ();
    float gzRate = rawGz - g_gzOffset;

    // Filtro de banda morta para ruído estático (< 0.2 dps)
    if (abs(gzRate) < 0.20f) {
        gzRate = 0.0f;
    }

    g_yawAngleDeg += gzRate * dt;

    if (now - lastPrint >= 150) {
        lastPrint = now;
        const char* sentido = "PARADO";
        if (gzRate > 3.0f) sentido = "GIRANDO (+) ";
        else if (gzRate < -3.0f) sentido = "GIRANDO (-) ";

        Serial.printf("[IMU] Taxa Z: %+6.1f deg/s | %s | Yaw Integrado: %+6.1f GRAUS\n",
                      gzRate, sentido, g_yawAngleDeg);
    }

    delay(10);
}
