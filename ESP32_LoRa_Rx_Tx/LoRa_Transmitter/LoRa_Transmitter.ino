#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

#define ss 18
#define rst 14
#define dio0 26
#define DHTPIN 0
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
 
int counter = 0;

void setup() {
  Serial.begin(9600);
   dht.begin(); //Im Setup wird der DHT11 gestartet
  //while (!Serial);
  Serial.println("LoRa Sender");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(ss, rst, dio0);
  
  if (!LoRa.begin(868E6)) {
    Serial.println("Start of LoRa failed!");
    while (1);
  }
}

void loop() {
   int humidity_data = (int)dht.readHumidity();
  int temperature_data = (int)dht.readTemperature();
  Serial.print("Sending packet: ");
  Serial.println(counter);
  // send packet
  LoRa.beginPacket();
  LoRa.print("Temp: ");
  LoRa.print(temperature_data);
   LoRa.print("  Hum: ");
  LoRa.print(humidity_data);
  LoRa.endPacket();
  delay(5000);
}
