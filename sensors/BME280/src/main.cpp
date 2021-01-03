#include <Arduino.h>
#include <Wire.h>     // Library für I2C nötig
#include <BME280_t.h> // Die BME280_Lite Library importieren

#define ASCII_ESC 27

#define MYALTITUDE 446.00 // Meereshöhe Klagenfurt

char bufout[10];

BME280<> BMESensor; // Sensor intialisieren

void setup()
{
  Serial.begin(9600); // Serrielle Baud-Rate definieren und initialisieren
  Wire.begin();       // I2C Pins vom Board definieren (SDA,SCL)
  BMESensor.begin();  // initalisiere BME280 Sensor
  BMESensor.seaLevelForAltitude(MYALTITUDE);
}

void loop()
{
  BMESensor.refresh(); // Gelesene Sensor Daten aktualisieren

  Serial.print("Temperature: ");
  Serial.print(BMESensor.temperature); // Serielle Ausgabe der gemessenen Temperatur
  Serial.println("C");

  Serial.print("Humidity:    ");
  Serial.print(BMESensor.humidity); // Gemessene Luftfeuchtigkeit in %
  Serial.println("%");

  Serial.print("Pressure:    ");
  Serial.print(BMESensor.pressure / 100.0F); // Gemessener Druck in hPa
  Serial.println("hPa");

  float altitude = BMESensor.pressureToAltitude();
  Serial.print("Altitude:    ");
  Serial.print(altitude); // Ungefähre Meereshöhe abhängig vom Luftdruck
  Serial.println("m");

  delay(1000); // #nohomo
  Serial.println("");
}
