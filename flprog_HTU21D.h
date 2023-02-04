#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"

#define FLPROG_HTU_HTDU21D_ADDRESS 0x40 // Unshifted 7-bit I2C address for the sensor

#define FLPROG_HTU_TRIGGER_TEMP_MEASURE_HOLD 0xE3
#define FLPROG_HTU_TRIGGER_HUMD_MEASURE_HOLD 0xE5
#define FLPROG_HTU_TRIGGER_TEMP_MEASURE_NOHOLD 0xF3
#define FLPROG_HTU_TRIGGER_HUMD_MEASURE_NOHOLD 0xF5
#define FLPROG_HTU_WRITE_USER_REG 0xE6
#define FLPROG_HTU_READ_USER_REG 0xE7
#define FLPROG_HTU_SOFT_RESET 0xFE
#define FLPROG_HTU_SHIFTED_DIVISOR 0x988000

#define FLPROG_HTU_WAITING_READ_STEP 0
#define FLPROG_HTU_WAITING_DELAY 1
#define FLPROG_HTU_READ_SENSOR_STEP1 2
#define FLPROG_HTU_READ_SENSOR_STEP2 3

#define FLPROG_HTU_NOT_ERROR 0
#define FLPROG_HTU_DEVICE_NOT_CORRECT_DATA_ERROR 70
#define FLPROG_HTU_DEVICE_NOT_CORRECT_DATA_SIZE_ERROR 71
#define FLPROG_HTU_CRC_ERROR 72

class FLProgHTU21D
{
public:
    FLProgHTU21D(FLProgI2C *device);
    void pool();
    float getHumidity() { return hum; };
    float getTemperature() { return temper; };
    uint8_t getError() { return codeError; };
    void resolution(uint8_t resBits);
    void read();
    void setReadPeriod(uint32_t period);

private:
    void checkDelay();
    void readSensor();
    void readSensorStep1();
    void readSensorStep2();
    byte read_user_register();
    byte check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor);
    void setResolution();
    void createError();
    FLProgI2C *i2cDevice;
    uint8_t step = FLPROG_HTU_WAITING_READ_STEP;
    uint8_t codeError = FLPROG_HTU_NOT_ERROR;
    uint32_t startDelay;
    uint32_t sizeDelay;
    uint8_t stepAfterDelay;
    bool isNeededRead = true;
    float temper = 0;
    float hum = 0;
    uint8_t curenrResolution = 0;
    uint8_t newResBits = 0;
    uint32_t readPeriod = 0;
    uint32_t startReadPeriod = 0;
};