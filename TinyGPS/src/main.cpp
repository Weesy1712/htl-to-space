#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}
void displayINFO();
void loop()
{
  while (Serial2.available())
  {
    char val = Serial2.read();
   // Serial.write(val);
    gps.encode(val);
  }

  displayINFO();
}

long nextTimeToExecuteDisplay = 0;
int delayValDisplay = 1000;
void displayINFO()
{
  if (millis() > nextTimeToExecuteDisplay)
  {
    Serial.print(F("Location:"));

    Serial.print(gps.location.lat(), 8);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 8);

    Serial.println();
    nextTimeToExecuteDisplay = millis() + delayValDisplay;
  }
}
