#include "flprog_HTU21D.h"

FLProgHTU21D::FLProgHTU21D(FLProgI2C *device)
{
    i2cDevice = device;
}

void FLProgHTU21D::checkDelay()
{
    if (flprog::isTimer(startDelay, sizeDelay))
    {
        step = stepAfterDelay;
    }
}

void FLProgHTU21D::pool()
{
    if (step == FLPROG_HTU_WAITING_DELAY)
    {
        checkDelay();
    }
    if (step == FLPROG_HTU_WAITING_READ_STEP)
    {
        if (newResBits != curenrResolution)
        {
        }
        else
        {
            if (isNeededRead)
            {
                readSensor();
                isNeededRead = false;
            }
            else
            {
                return;
            }
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

void FLProgHTU21D::readSensor()
{
    i2cDevice->beginTransmission(FLPROG_HTU_HTDU21D_ADDRESS);
    i2cDevice->write(FLPROG_HTU_TRIGGER_HUMD_MEASURE_NOHOLD); // Measure humidity with no bus holding
    codeError = i2cDevice->endTransmission();
    if (codeError)
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        return;
    }
    startDelay = millis();
    sizeDelay = 55;
    stepAfterDelay = FLPROG_HTU_READ_SENSOR_STEP1;
    step = FLPROG_HTU_WAITING_DELAY;
}

void FLProgHTU21D::readSensorStep1()
{
    i2cDevice->requestFrom(FLPROG_HTU_HTDU21D_ADDRESS, 3);
    if (i2cDevice->waitingForData(3))
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        codeError = FLPROG_HTU_DEVICE_NOT_CORRECT_DATA_ERROR;
        return;
    }
    byte msb, lsb, checksum;
    msb = i2cDevice->read();
    lsb = i2cDevice->read();
    checksum = i2cDevice->read();
    unsigned int rawHumidity = ((unsigned int)msb << 8) | (unsigned int)lsb;
    if (check_crc(rawHumidity, checksum) != 0)
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        codeError = FLPROG_HTU_CRC_ERROR;
        return;
    }
    rawHumidity &= 0xFFFC;
    float tempRH = rawHumidity / (float)65536;
    hum = -6 + (125 * tempRH);
    i2cDevice->beginTransmission(FLPROG_HTU_HTDU21D_ADDRESS);
    i2cDevice->write(FLPROG_HTU_TRIGGER_TEMP_MEASURE_NOHOLD);
    codeError = i2cDevice->endTransmission();
    if (codeError)
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        return;
    }
    startDelay = millis();
    sizeDelay = 55;
    stepAfterDelay = FLPROG_HTU_READ_SENSOR_STEP2;
    step = FLPROG_HTU_WAITING_DELAY;
}

void FLProgHTU21D::readSensorStep2()
{
    i2cDevice->requestFrom(FLPROG_HTU_HTDU21D_ADDRESS, 3);
    if (i2cDevice->waitingForData(3))
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        codeError = FLPROG_HTU_DEVICE_NOT_CORRECT_DATA_ERROR;
        return;
    }
    unsigned char msb, lsb, checksum;
    msb = i2cDevice->read();
    lsb = i2cDevice->read();
    checksum = i2cDevice->read();
    unsigned int rawTemperature = ((unsigned int)msb << 8) | (unsigned int)lsb;
    if (check_crc(rawTemperature, checksum) != 0)
    {
        step = FLPROG_HTU_WAITING_READ_STEP;
        codeError = FLPROG_HTU_CRC_ERROR;
        return;
    }
    rawTemperature &= 0xFFFC;
    float tempTemperature = rawTemperature / (float)65536;
    temper = -46.85 + (175.72 * tempTemperature);
    codeError = FLPROG_HTU_NOT_ERROR;
    step = FLPROG_HTU_WAITING_READ_STEP;
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
    byte userRegister;
    i2cDevice->beginTransmission(FLPROG_HTU_HTDU21D_ADDRESS);
    i2cDevice->write(FLPROG_HTU_READ_USER_REG);
    codeError = i2cDevice->endTransmission();
    if (codeError)
    {
        startDelay = millis();
        sizeDelay = 500;
        stepAfterDelay = FLPROG_HTU_WAITING_READ_STEP;
        step = FLPROG_HTU_WAITING_DELAY;
        return 0;
    }
    i2cDevice->requestFrom(FLPROG_HTU_HTDU21D_ADDRESS, 1);
    if (i2cDevice->waitingForData(1))
    {
        codeError = FLPROG_HTU_DEVICE_NOT_CORRECT_DATA_SIZE_ERROR;
        startDelay = millis();
        sizeDelay = 500;
        stepAfterDelay = FLPROG_HTU_WAITING_READ_STEP;
        step = FLPROG_HTU_WAITING_DELAY;
        return 0;
    }
    userRegister = i2cDevice->read();
    return userRegister;
}

void FLProgHTU21D::read()
{
    isNeededRead = true;
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
    i2cDevice->beginTransmission(FLPROG_HTU_HTDU21D_ADDRESS);
    i2cDevice->write(FLPROG_HTU_WRITE_USER_REG);
    i2cDevice->write(userRegister);
    codeError = i2cDevice->endTransmission();
    if (codeError)
    {
        startDelay = millis();
        sizeDelay = 500;
        stepAfterDelay = FLPROG_HTU_WAITING_READ_STEP;
        step = FLPROG_HTU_WAITING_DELAY;
        return;
    }
    curenrResolution = newResBits;
}