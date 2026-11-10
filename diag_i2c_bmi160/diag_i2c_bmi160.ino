/*
 * diag_i2c_bmi160.ino
 * Diagnostico I2C e Leitura do Sensor IMU BMI160
 *
 * Conexoes I2C (ESP32):
 *  SDA -> GPIO 21
 *  SCL -> GPIO 22
 *  VCC -> 3.3V (ou 5V se o modulo tiver regulador embutido)
 *  GND -> GND
 *
 * O BMI160 possui dois enderecos I2C possiveis:
 *  0x68 (padrao quando SDO/SA0 = GND)
 *  0x69 (quando SDO/SA0 = VCC)
 *
 * Registradores Importantes do BMI160:
 *  0x00: CHIP_ID (espera-se ler 0xD8)
 *  0x7E: CMD (0x11 liga acelerometro, 0x15 liga giroscopio, 0xB6 faz soft-reset)
 *  0x02: Erros
 *  0x03: Status de PMU
 *  0x0C - 0x11: Dados de Giroscopio (X, Y, Z) - 16 bits cada
 *  0x12 - 0x17: Dados de Acelerometro (X, Y, Z) - 16 bits cada
 */

#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define BMI160_CHIP_ID_REG  0x00
#define BMI160_CMD_REG      0x7E
#define BMI160_DATA_REG     0x0C // Inicio dos dados (Gyr X_LSB)

uint8_t bmi160_addr = 0;

void i2cScanner() {
    Serial.println("\n-------------------------------------------");
    Serial.println("  Iniciando I2C Bus Scanner (SDA=21, SCL=22)");
    Serial.println("-------------------------------------------");
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf(" [OK] Dispositivo I2C encontrado no endereco 0x%02X", address);
            if (address == 0x68) {
                Serial.print("  <-- Possivel BMI160 (SDO no GND)");
                bmi160_addr = 0x68;
            } else if (address == 0x69) {
                Serial.print("  <-- Possivel BMI160 (SDO no VCC)");
                bmi160_addr = 0x69;
            }
            Serial.println();
            nDevices++;
        } else if (error == 4) {
            Serial.printf(" [ERRO] Erro desconhecido no endereco 0x%02X\n", address);
        }
    }

    if (nDevices == 0) {
        Serial.println(" [FALHA] Nenhum dispositivo I2C encontrado!");
        Serial.println(" -> Verifique a alimentacao (VCC/GND)");
        Serial.println(" -> Verifique se SDA esta no GPIO 21 e SCL no GPIO 22");
    } else {
        Serial.printf("Varredura concluida. %d dispositivo(s) encontrado(s).\n\n", nDevices);
    }
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool initBMI160(uint8_t addr) {
    // 1. Verificar Chip ID (Padrao oficial do BMI160 no registrador 0x00 e 0xD1)
    uint8_t chip_id = readRegister(addr, BMI160_CHIP_ID_REG);
    Serial.printf("[BMI160] Chip ID lido no endereco 0x%02X: 0x%02X\n", addr, chip_id);

    if (chip_id != 0xD1) {
        Serial.printf("[FALHA] Chip ID incompativel! Esperado: 0xD1, Lido: 0x%02X\n", chip_id);
        return false;
    }
    Serial.println("[OK] Chip ID 0xD1 confirmado com sucesso! Sensor e genuino BMI160.");

    // 2. Soft-reset no sensor (comando 0xB6)
    writeRegister(addr, BMI160_CMD_REG, 0xB6);
    delay(100);

    // 3. Ligar Acelerometro no modo Normal (CMD: 0x11)
    writeRegister(addr, BMI160_CMD_REG, 0x11);
    delay(50);

    // 4. Ligar Giroscopio no modo Normal (CMD: 0x15)
    writeRegister(addr, BMI160_CMD_REG, 0x15);
    delay(80);

    // 5. Configurar Range do Acelerometro para +-2G (Reg 0x41 = 0x03)
    writeRegister(addr, 0x41, 0x03);
    delay(10);

    // 6. Configurar Range do Giroscopio para +-250 deg/s (Reg 0x43 = 0x03)
    writeRegister(addr, 0x43, 0x03);
    delay(10);

    // 7. Configurar ODR (Output Data Rate) do Acc para 100Hz (Reg 0x40 = 0x28)
    writeRegister(addr, 0x40, 0x28);
    // 8. Configurar ODR do Gyr para 100Hz (Reg 0x42 = 0x28)
    writeRegister(addr, 0x42, 0x28);
    delay(20);

    uint8_t pmu_status = readRegister(addr, 0x03);
    Serial.printf("[OK] Acelerometro e Giroscopio ativados. PMU Status (0x03): 0x%02X\n", pmu_status);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==============================================");
    Serial.println("    DIAGNOSTICO E LEITURA DA IMU BMI160       ");
    Serial.println("==============================================");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz Fast Mode

    i2cScanner();

    if (bmi160_addr != 0) {
        if (!initBMI160(bmi160_addr)) {
            Serial.println("[ERRO] Falha ao configurar registradores do BMI160.");
        }
    } else {
        Serial.println("[AVISO] Como o endereco do BMI160 nao foi detectado automaticamente,");
        Serial.println("        tentando comunicar forcando 0x68...");
        initBMI160(0x68);
    }

    Serial.println("----------------------------------------------\n");
}

unsigned long lastRead = 0;

void loop() {
    if (bmi160_addr == 0) {
        delay(1000);
        return;
    }

    unsigned long now = millis();
    if (now - lastRead >= 200) { // 5 Hz de telemetria
        lastRead = now;

        // Leitura em rajada (burst read) dos 12 bytes de dados (Giroscopio + Acelerometro)
        Wire.beginTransmission(bmi160_addr);
        Wire.write(BMI160_DATA_REG);
        Wire.endTransmission(false);

        // 12 bytes: GyrX (2), GyrY (2), GyrZ (2), AccX (2), AccY (2), AccZ (2)
        uint8_t count = Wire.requestFrom((uint8_t)bmi160_addr, (uint8_t)12);
        if (count == 12) {
            int16_t gyr_x = Wire.read() | (Wire.read() << 8);
            int16_t gyr_y = Wire.read() | (Wire.read() << 8);
            int16_t gyr_z = Wire.read() | (Wire.read() << 8);

            int16_t acc_x = Wire.read() | (Wire.read() << 8);
            int16_t acc_y = Wire.read() | (Wire.read() << 8);
            int16_t acc_z = Wire.read() | (Wire.read() << 8);

            // Escala +-2G -> 16384 LSB/g
            float ax_g = (float)acc_x / 16384.0f;
            float ay_g = (float)acc_y / 16384.0f;
            float az_g = (float)acc_z / 16384.0f;

            // Escala +-250 deg/s -> 131.2 LSB/(deg/s)
            float gx_dps = (float)gyr_x / 131.2f;
            float gy_dps = (float)gyr_y / 131.2f;
            float gz_dps = (float)gyr_z / 131.2f;

            Serial.printf("[BMI160] ACC (g): X=%+5.2f Y=%+5.2f Z=%+5.2f | GYR (deg/s): X=%+6.1f Y=%+6.1f Z=%+6.1f\n",
                          ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        } else {
            Serial.printf("[AVISO] Falha ao ler bytes do sensor (recebidos: %d)\n", count);
        }
    }
}
