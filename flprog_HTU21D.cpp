#include "flprog_HTU21D.h"

FLProgHTU21D::FLProgHTU21D(AbstractFLProgI2C *device)
{
    i2cDevice = device;
    addres = 0x40;
}

void FLProgHTU21D::pool()
{
    checkReadPeriod();
    checkDelay();
    if (step == FLPROG_SENSOR_WAITING_READ_STEP)
    {
        if (newResBits != curenrResolution)
        {
        }
        else
        {
            checkNeededRead();
        }
    }
    if (step == FLPROG_HTU_READ_SENSOR_STEP1)
    {
        readSensorStep1();
    }
    if (step == FLPROG_HTU_READ_SENSOR_STEP2)
    {
        readSensorStep2();
    }
}

void FLProgHTU21D::createError()
{
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 500);
}

void FLProgHTU21D::readSensor()
{
    codeError = i2cDevice->fullWrite(addres, FLPROG_HTU_TRIGGER_HUMD_MEASURE_NOHOLD);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_HTU_READ_SENSOR_STEP1, 55);
}

void FLProgHTU21D::readSensorStep1()
{
    uint8_t temp[3];
    codeError = i2cDevice->fullRead(addres, temp, 3);
    if (codeError)
    {
        createError();
        return;
    }
    unsigned int rawHumidity = ((unsigned int)(temp[0]) << 8) | (unsigned int)(temp[1]);
    if (check_crc(rawHumidity, (temp[2])) != 0)
    {
        step = FLPROG_SENSOR_WAITING_READ_STEP;
        codeError = FLPROG_SENSOR_CRC_ERROR;
        return;
    }
    rawHumidity &= 0xFFFC;
    float tempRH = rawHumidity / (float)65536;
    hum = -6 + (125 * tempRH);
    codeError = i2cDevice->fullWrite(addres, FLPROG_HTU_TRIGGER_TEMP_MEASURE_NOHOLD);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_HTU_READ_SENSOR_STEP2, 55);
}

void FLProgHTU21D::readSensorStep2()
{
    uint8_t temp[3];
    codeError = i2cDevice->fullRead(addres, temp, 3);
    if (codeError)
    {
        createError();
        return;
    }
    unsigned int rawTemperature = ((unsigned int)(temp[0]) << 8) | (unsigned int)(temp[1]);
    if (check_crc(rawTemperature, (temp[2])) != 0)
    {
        step = FLPROG_SENSOR_WAITING_READ_STEP;
        codeError = FLPROG_SENSOR_CRC_ERROR;
        return;
    }
    rawTemperature &= 0xFFFC;
    float tempTemperature = rawTemperature / (float)65536;
    temper = -46.85 + (175.72 * tempTemperature);
    codeError = FLPROG_SENSOR_NOT_ERROR;
    step = FLPROG_SENSOR_WAITING_READ_STEP;
}

byte FLProgHTU21D::check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
    uint32_t remainder = (uint32_t)message_from_sensor << 8;
    remainder |= check_value_from_sensor;
    uint32_t divsor = (uint32_t)FLPROG_HTU_SHIFTED_DIVISOR;
    for (int i = 0; i < 16; i++)
    {
        if (remainder & (uint32_t)1 << (23 - i))
            remainder ^= divsor;

        divsor >>= 1;
    }
    return (byte)remainder;
}

byte FLProgHTU21D::read_user_register()
{
    uint8_t userRegister;
    codeError = i2cDevice->fullWrite(addres, FLPROG_HTU_READ_USER_REG);
    if (codeError)
    {
        createError();
        return 0;
    }
    if (i2cDevice->fullRequestFrom(addres, 1))
    {
        createError();
        return 0;
    }
    userRegister = i2cDevice->read();
    return userRegister;
}


void FLProgHTU21D::resolution(uint8_t resBits)
{
    newResBits = resBits;
}

// Set sensor resolution
/*******************************************************************************************/
// Sets the sensor resolution to one of four levels
// Page 12:
//  0/0 = 12bit RH, 14bit Temp
//  0/1 = 8bit RH, 12bit Temp
//  1/0 = 10bit RH, 13bit Temp
//  1/1 = 11bit RH, 11bit Temp
// Power on default is 0/0
void FLProgHTU21D::setResolution()
{
    byte userRegister = read_user_register();
    byte temp = newResBits;
    if (codeError)
    {
        return;
    }
    userRegister &= 0b01111110;
    temp &= 0b10000001;
    userRegister |= temp;
    i2cDevice->beginTransmission(addres);
    i2cDevice->write(FLPROG_HTU_WRITE_USER_REG);
    i2cDevice->write(userRegister);
    codeError = i2cDevice->endTransmission();
    if (codeError)
    {
        createError();
        return;
    }
    curenrResolution = newResBits;
}
