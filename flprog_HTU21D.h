#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"
#include "flprogI2C.h"


#define FLPROG_HTU_TRIGGER_TEMP_MEASURE_HOLD 0xE3
#define FLPROG_HTU_TRIGGER_HUMD_MEASURE_HOLD 0xE5
#define FLPROG_HTU_TRIGGER_TEMP_MEASURE_NOHOLD 0xF3
#define FLPROG_HTU_TRIGGER_HUMD_MEASURE_NOHOLD 0xF5
#define FLPROG_HTU_WRITE_USER_REG 0xE6
#define FLPROG_HTU_READ_USER_REG 0xE7
#define FLPROG_HTU_SOFT_RESET 0xFE
#define FLPROG_HTU_SHIFTED_DIVISOR 0x988000

#define FLPROG_HTU_READ_SENSOR_STEP1 10
#define FLPROG_HTU_READ_SENSOR_STEP2 11




class FLProgHTU21D : public FLProgI2cStepWorkSensor
{
public:
    FLProgHTU21D(AbstractFLProgI2C *device);
    void pool();
    float getHumidity() { return hum; };
    float getTemperature() { return temper; };
    void resolution(uint8_t resBits);
   

protected:
  
    virtual void readSensor();
    void readSensorStep1();
    void readSensorStep2();
    byte read_user_register();
    byte check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor);
    void setResolution();
    void createError();
    float temper = 0;
    float hum = 0;
    uint8_t curenrResolution = 0;
    uint8_t newResBits = 0;
};