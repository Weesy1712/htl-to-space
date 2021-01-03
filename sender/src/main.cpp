#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <LoraMessage.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <OneWire.h>

#include <SD.h>
File sdCard;

#define ONE_WIRE_BUS 17
OneWire oneWire(ONE_WIRE_BUS);

/*Sensors*/
// BME280
#include <Wire.h>     // Library für I2C nötig
#include <BME280_t.h> // Die BME280_Lite Library importieren
#define ASCII_ESC 21
#define MYALTITUDE 446.00 // Meereshöhe Klagenfurt
char bufout[10];
BME280<> BMESensor; // Barometer intialisieren
// GY1146
Adafruit_SI1145 uv = Adafruit_SI1145(); // Lichtitensitätssensor inizialisieren
// 9 DOF
/* Assign a unique ID to the sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Initialises all the sensors used by this example

#define BUILTIN_LED 25

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/15, /* data=*/4, /* reset=*/16);

// TODO reset keys

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x84, 0x5A, 0xE0, 0x76, 0xD9, 0x63, 0xEF, 0xA3, 0x28, 0x03, 0xD4, 0x0E, 0x66, 0x44, 0x16, 0x2E };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xBC, 0xDA, 0xAD, 0xA4, 0xC3, 0xBE, 0xC1, 0x74, 0x24, 0x43, 0xC3, 0xA4, 0x0F, 0xA3, 0xEA, 0xD6 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260116DE; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

void do_send(osjob_t *j);

LoraMessage message;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  u8x8.setCursor(0, 6);
  u8x8.printf("TIME %lu", os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    u8x8.drawString(0, 7, "EV_BEACON_FOUND");
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    u8x8.drawString(0, 7, "EV_BEACON_MISSED");
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    u8x8.drawString(0, 7, "EV_JOINING");
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    u8x8.drawString(0, 7, "EV_JOINED ");
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    u8x8.drawString(0, 7, "EV_RFUI");
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    u8x8.drawString(0, 7, "EV_JOIN_FAILED");
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    u8x8.drawString(0, 7, "EV_TXCOMPLETE");
    digitalWrite(BUILTIN_LED, LOW);
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      Serial.println(F("Received ack"));
      u8x8.drawString(0, 7, "Received ACK");
    }
    if (LMIC.dataLen)
    {
      Serial.println(F("Received "));
      u8x8.drawString(0, 6, "RX ");
      Serial.println(LMIC.dataLen);
      u8x8.setCursor(4, 6);
      u8x8.printf("%i bytes", LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      u8x8.setCursor(0, 7);
      u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    u8x8.drawString(0, 7, "EV_LOST_TSYNC");
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    u8x8.drawString(0, 7, "EV_RESET");
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    u8x8.drawString(0, 7, "EV_RXCOMPLETE");
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    u8x8.drawString(0, 7, "EV_LINK_DEAD");
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    u8x8.drawString(0, 7, "EV_LINK_ALIVE");
    break;
  default:
    Serial.println(F("Unknown event"));
    u8x8.setCursor(0, 7);
    u8x8.printf("UNKNOWN EVENT %d", ev);
    break;
  }
}

unsigned int lastTimeSent = 0;

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    if (message.getLength() > 0)
    {
      if (millis() > lastTimeSent + 110000)
      {
        //Serial.println(message.getLength());
        LMIC_setTxData2(1, (uint8_t *)message.getBytes(), message.getLength(), 0);
        Serial.println(F("Packet queued"));
        u8x8.drawString(0, 7, "PACKET QUEUED");
        digitalWrite(BUILTIN_LED, HIGH);

        sdCard.close();
        sdCard = SD.open("lora.txt");

        lastTimeSent = millis();
      }
      else
      {
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      }
    }
    else
    {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      Serial.println(F("No Data to transmit"));
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

float temperature;

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Wire.begin(21, 22); // I2C Pins vom Board definieren (SDA,SCL)
  /*Sensors*/
  // BME280
  if (!BMESensor.begin())
  {
    Serial.println("Didn't find BME");
    while (1)
      ;
  }
  BMESensor.seaLevelForAltitude(MYALTITUDE); // Meereshöhe kalibrieren
  
  // GY1145
  if (!uv.begin())
  {
    Serial.println("Didn't find Si1145");
    while (1)
      ;
  }
  // 9 DOF
   if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1)
      ;
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

if (!SD.begin())
  {
    Serial.println("Didn't find SD");
    while (1)
      ;
  }
  sdCard = SD.open("lora.txt", FILE_WRITE);
  sdCard.println("Timestamp;Temperature;Humidity;Pressure;Altitude;VisibleLight;InfraredLight;UltraViolettLight;X;Y;Z");

  // Display
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  SPI.begin(5, 19, 27);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif
#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

// TTN defines an additional channel at 869.525Mhz using SF9 for class B
// devices' ping slots. LMIC does not have an easy way to define set this
// frequency and support for class B is spotty and untested, so this
// frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Start job
  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

void BME(LoraMessage *temp)
{
  BMESensor.refresh(); // Gelesene Sensor Daten aktualisieren

  float temperature = BMESensor.temperature;
  float humidity = BMESensor.humidity;
  float pressure = BMESensor.pressure;
  float alt = BMESensor.pressureToAltitude();

  Serial.print("Temperatur: ");
  Serial.print(temperature);
  Serial.print("  Humidity: ");
  Serial.print(humidity);
  Serial.print("  Pressure: ");
  Serial.println(pressure);
  Serial.print("  Altitude: ");
  Serial.println(alt);
  Serial.println("-------------------------");

  temp->addTemperature(temperature);
  temp->addHumidity(humidity);
  temp->addUnixtime((uint32_t)pressure);
  temp->addUint16((uint16_t)alt);

  char buff[100];
  snprintf(buff, sizeof(buff), "%f;%f;%f;%f;", temperature, humidity, pressure, alt);
  String bmeData = buff;
  sdCard.print(bmeData);
}

void GY(LoraMessage *temp)
{
  int vl = uv.readVisible();
  int ir = uv.readIR();
  int uvl = uv.readUV();

  temp->addUint16(vl);
  temp->addUint16(ir);
  temp->addUint16(uvl);

  Serial.print("Sichtbares Licht: ");
  Serial.print(vl);
  Serial.print("  Infrarot: ");
  Serial.print(ir);
  Serial.print("  Ultra Violett: ");
  Serial.println(uvl);
  Serial.println("-------------------------");

  char buff[100];
  snprintf(buff, sizeof(buff), "%d;%d;%d;%d;", vl, ir, uvl);
  String gyData = buff;
  sdCard.print(gyData);
}

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t orientation;

void DOF(LoraMessage *temp)
{
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */

    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));

    temp->addTemperature(orientation.roll);
    temp->addTemperature(orientation.pitch);

    char buff[100];
    snprintf(buff, sizeof(buff), "%f;%f;", orientation.roll, orientation.pitch);
    String gyData = buff;
    sdCard.print(gyData);
  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */

    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));

    temp->addTemperature(orientation.heading);

    char buff[100];
    snprintf(buff, sizeof(buff), "%f;", orientation.heading);
    String gyData = buff;
    sdCard.print(gyData);
  }
}

char temp[20];
unsigned long nextTimeToExecute;
unsigned long delta = TX_INTERVAL * 500;

void loop()
{
  if (nextTimeToExecute <= millis())
  {
    LoraMessage temp;

    sdCard.print(millis() + "");

    /*Sensors*/
    BME(&temp);
    GY(&temp);
    DOF(&temp);
    sdCard.print("\n");

    // TODO: Sensordaten auf SD Karte.


    Serial.println("\n=========================");
    message = temp;
    os_runloop_once();

    nextTimeToExecute = millis() + delta;
  }

  // Sensordaten auf den Display schreiben
  
  snprintf(temp, 20, "Temp:  %5.2f C", BMESensor.temperature);
  u8x8.drawString(0, 1, temp);
  snprintf(temp, 20, "Alt:  %5.2f m  ", BMESensor.pressureToAltitude());
  u8x8.drawString(0, 2, temp);

  snprintf(temp, 20, "Licht: %5.2i", uv.readVisible());
  u8x8.drawString(0, 3, temp);
  snprintf(temp, 20, "IR/UV:%3.2i/%2.2i", uv.readIR(), uv.readUV());
  u8x8.drawString(0, 4, temp);
  snprintf(temp, 20, "X:     %2.2f", orientation.roll);
  u8x8.drawString(0, 5, temp);
  snprintf(temp, 20, "Y:     %3.2f", orientation.pitch);
  u8x8.drawString(0, 6, temp);
  snprintf(temp, 20, "Z:    %3.2f", orientation.heading);
  u8x8.drawString(0, 7, temp);
}