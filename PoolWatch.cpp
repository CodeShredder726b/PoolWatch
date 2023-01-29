/* MKR WAN 1310 LoRa module. */
#include <MKRWAN.h>
#include <ArduinoLowPower.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include "keys.h"

//#define LOW_POWER_EN
//#define LORA_SEND_EN
//#define BAT_EN

/* PH */
/**
 * Calibration:
 * - Rohrreiniger   12-14
 * - Wasser         7-9
 * - Milch          6
 * - Kaffee         5
 * - Essig          2
 * 
 * Measure two defined pH solutions, add them to ph1/v1 and ph2/v2
 */
#define v1Calib           (0.67)
#define ph1Calib          (2)
#define v2Calib           (2.01)
#define ph2Calib          (8)
/* calculate offset */
#define offset            (ph1Calib - ((ph2Calib - ph1Calib) / (v2Calib - v1Calib)) * v1Calib)

/* Hardware */
#define PH_ANALOG_PIN     (A0)
#define DS18B20_PIN       (0)

/* Sleep for 60*60*1000 ms = 1 hour*/
#define SLEEP_MS          (60 * 60 * 1000)

/* LoRa */
static const uint8_t join_mode = 1; //1:OTAA, 2:ABP
LoRaModem modem;

/* DS18B20 Temperature */
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

/* AHT20 Temp/RH */
Adafruit_AHTX0 aht;

#ifdef BAT_EN
#define PAYLOAD_SIZE    (10)
#else
#define PAYLOAD_SIZE    (8)
#endif
byte payload[PAYLOAD_SIZE];

void getData()
{
  uint32_t ph = 0;
  uint32_t tempWater = 0;
  uint32_t tempBoard = 0;
  uint32_t rh = 0;
  #ifdef BAT_EN
  uint32_t bat = 0;
  #endif
  
  /*
  uint64_t analog_measure = 0;
  for (int i = 0; i < AVG; i++)
  {
    analog_measure += analogRead(PH_ANALOG_PIN);
    delay(100);
  }

  //analog_measure = analogRead(PH_ANALOG_PIN);
  analog_measure = analog_measure / AVG;
  Serial.println(analog_measure);                     //              697 (2238mV)

  float ph_mv = (analog_measure / AVG) * 5.0 / 1024;  // to milivolts 3.41
  Serial.println(ph_mv);

  int32_t ph = 3.5 * (ph_mv + PH_OFFSET) * 1000;    // to ph         0.028
  Serial.println(ph);
  */

  /* read analog value */
  analogReadResolution(12);
  int a = analogRead(PH_ANALOG_PIN);
  
  /* 3.3V Range / 4095 Resolution * analog value */
  float V = 3.3 / 4095 * a;
  Serial.print("measured [V]: ");
  Serial.print(V);

  ph = (((ph2Calib - ph1Calib) / (v2Calib - v1Calib)) * V + offset) * 100;
  Serial.print("  -> pH: ");
  Serial.print(ph/100);
  Serial.println();

#ifdef BAT_EN
////////// TODO //////////
  bat = 3.5 * 100;    
////////// TODO //////////
#endif

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  Serial.print("Temperature Water [°C]: ");
  Serial.print(tempC);
  Serial.println();
  tempWater = (uint32_t)(tempC*100);

  sensors_event_t ahthumidity, ahttemp;
  aht.getEvent(&ahthumidity, &ahttemp);
  
  Serial.print("Temperature Board [°C]: ");
  Serial.print(ahttemp.temperature);
  Serial.println();
  tempBoard = (uint32_t)(ahttemp.temperature*100);

  Serial.print("RH [%]: ");
  Serial.print(ahthumidity.relative_humidity);
  Serial.println();
  tempBoard = (uint32_t)(ahthumidity.relative_humidity*100);
  
  payload[0] = highByte(ph);
  payload[1] = lowByte(ph);
  payload[2] = highByte(tempWater);
  payload[3] = lowByte(tempWater);
  payload[4] = highByte(tempBoard);
  payload[5] = lowByte(tempBoard);
  payload[6] = highByte(rh);
  payload[7] = lowByte(rh);
#ifdef BAT_EN
  payload[8] = highByte(bat);
  payload[9] = lowByte(bat);
#endif
}

void sendLora()
{
  int err;
  modem.setPort(3);
  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  err = modem.endPacket(true);

  memset(payload,0,sizeof(payload));
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // ph sensor analog input
  pinMode(PH_ANALOG_PIN, INPUT);

  // ds18b20 temp sensor 
  sensors.begin();

  if (! aht.begin()) {
    Serial.println("Could not find AHT20!");
    while(1) {
      delay(10);
    }
  }
  
  getData();

  if (!modem.begin(EU868))
  {
    Serial.println("Failed to start module");
    while (1) {}
  };

  Serial.print("Your module version is: ");
  Serial.println(modem.version());

  if (modem.version() != ARDUINO_FW_VERSION)
  {
    Serial.println("Update the firmware upload the 'MKRWANFWUpdate_standalone.ino' sketch.");
  }

  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected;
  do {
    Serial.println("Join");
    switch(join_mode) {
      case 1:
        connected = modem.joinOTAA(appEui, appKey);
      break;
      case 2:
        connected = modem.joinABP(devAddr, nwkSKey, appSKey);
      break;
      default:
        Serial.println("Join mode not defined. Use 1 (OTAA) or 2 (ABP).");
        while(1) {
          delay(10);
        }
        break;
    }

    if (!connected)
    {
      Serial.println("Connection failed. Try again...");
      delay(10000);
    }
  } while (!connected);
  
  delay(1000);
}

void loop()
{
  //  while (modem.available()) {
  //    Serial.write(modem.read());
  //  }
  //  modem.poll();
  digitalWrite(LED_BUILTIN, HIGH);

  getData();
  digitalWrite(LED_BUILTIN, LOW);
  
#ifdef LORA_SEND_EN
  sendLora();
#endif

#ifdef LOW_POWER_EN
  LowPower.sleep(sleep_ms); //or deepSleep
  // https://www.youtube.com/watch?v=urLSDi7SD8M
#else
  delay(10000);
#endif
}
