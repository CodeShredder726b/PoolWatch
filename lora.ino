/* MKR WAN 1310 LoRa module. */
#include <MKRWAN.h>
#include <ArduinoLowPower.h>

LoRaModem modem;

#define PH_ANALOG_PIN   (A0)
#define PH_OFFSET       (-5.8)//(-3.41)
#define AVG             (1)

String appEui   = "7061756C706F6F6C";
String appKey   = "953A6927DD6DFAB49AD447F9E8739F23";
String devAddr  = "260B66FE";
String nwkSKey  = "0C09AA8AA6A6254391E23E608BE1806D";
String appSKey  = "C05FACF5E67D31410ED7F7A7096EDB13";
int mode        = 1; //1:OTAA, 2:ABP
byte payload[8];
uint32_t sleep_ms = 60 * 60 * 1000; // 1 hour

void getData()
{
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

  uint32_t bat = 0;
  uint32_t temp = 0;

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
  modem.write(payload, 8);
  err = modem.endPacket(true);

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
