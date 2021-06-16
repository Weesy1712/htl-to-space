#include <SPI.h>
#include <LoRa.h>

#define ss 18
#define rst 14
#define dio0 26

String RxString;
String RxRSSI;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Rx Test");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(ss, rst, dio0);
  
  if (!LoRa.begin(868E6)) {
    Serial.println("Start of LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      RxString = (char)LoRa.read();
      Serial.print(RxString);
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    RxRSSI = LoRa.packetRssi();
    Serial.println(RxRSSI);
  }
}
