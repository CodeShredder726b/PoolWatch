/* MKR WAN 1310 LoRa module. */
#include <MKRWAN.h>
#include <ArduinoLowPower.h>
#include "keys.h"

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
#define v1Calib         (0.67)
#define ph1Calib        (2)
#define v2Calib         (2.01)
#define ph2Calib        (8)
/* calculate offset */
#define offset          (ph1Calib - ((ph2Calib - ph1Calib) / (v2Calib - v1Calib)) * v1Calib)

#define PH_ANALOG_PIN     (A0)

int phArray[8];

/* LoRa */
LoRaModem modem;

int mode = 1; //1:OTAA, 2:ABP
byte payload[8];

/* Application */
uint32_t sleep_ms = 30 * 1000; /* sleep in ms 60*60*1000 = 1 hour */
//uint32_t sleep_ms = 60 * 60 * 1000; /* sleep in ms 60*60*1000 = 1 hour */

void getData()
{
  uint32_t bat = 0;
  uint32_t temp = 0;
  uint32_t ph = 0;
  
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

  ph = (((ph2Calib - ph1Calib) / (v2Calib - v1Calib)) * V + offset) * 1000;
  Serial.print("  -> pH: ");
  Serial.print(ph/1000);
  Serial.println();

  payload[0] = highByte(ph);
  payload[1] = lowByte(ph);
  payload[2] = highByte(bat);
  payload[3] = lowByte(bat);
  payload[4] = highByte(temp);
  payload[5] = lowByte(temp);
  payload[6] = 0;
  payload[7] = 0;
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
    if (mode == 1) {
      connected = modem.joinOTAA(appEui, appKey);
    } else if (mode == 2) {
      connected = modem.joinABP(devAddr, nwkSKey, appSKey);
    }

    if (!connected)
    {
      Serial.println("Connection failed. Try again...");
      delay(10000);
    }
  } while (!connected);
}

void loop()
{
  //  while (modem.available()) {
  //    Serial.write(modem.read());
  //  }
  //  modem.poll();
  digitalWrite(LED_BUILTIN, HIGH);

  getData();
  sendLora();

  digitalWrite(LED_BUILTIN, LOW);
  //delay(10000);
  LowPower.sleep(sleep_ms); //or deepSleep
  // https://www.youtube.com/watch?v=urLSDi7SD8M
}
