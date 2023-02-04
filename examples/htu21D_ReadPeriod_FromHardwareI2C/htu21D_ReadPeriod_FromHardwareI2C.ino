#include "flprog_HTU21D.h"

FLProgI2C wireDevice(0);
FLProgHTU21D sensor(&wireDevice);
uint32_t startTime;
uint32_t startTime1;
uint32_t maxCicleTime = 0;
uint32_t startCicleTime = 0;
uint32_t cicleTime = 0;

void setup()
{
    Serial.begin(9600);
    wireDevice.begin();
    sensor.setReadPeriod(1000);
    startTime = millis() + 3000;
    startTime1 = millis() + 3000;
}

void loop()
{
    sensor.pool();
    if (flprog::isTimer(startTime, 1000))
    {
        Serial.print("Temperatura - ");
        Serial.println(sensor.getTemperature());
        Serial.print("Hum - ");
        Serial.println(sensor.getHumidity());
        Serial.print("Error - ");
        Serial.println(sensor.getError());
        Serial.print("maxCicleTime - ");
        Serial.println(maxCicleTime);
        Serial.println();
        startTime = millis();
    }
    else
    {
        if (flprog::isTimer(startTime1, 2000))
        {
            startCicleTime = micros();
            cicleTime = micros() - startCicleTime;
            maxCicleTime = max(maxCicleTime, cicleTime);
        }
    }
}