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

#define phSensorPin     (A0)

int phArray[8];

void setup() {
  Serial.begin(9600);
  Serial.println("ph sensor");
  Serial.println(offset);
}

void loop() {
  analogReadResolution(12);
  int a = analogRead(phSensorPin);

  /* 3.3V Range / 4095 Resolution * analog value */
  float V = 3.3 / 4095 * a;
  Serial.print("measured [V]: ");
  Serial.print(V);

  float ph = ((ph2Calib - ph1Calib) / (v2Calib - v1Calib)) * V + offset;
  Serial.print("  -> pH: ");
  Serial.print(ph);

  Serial.println();
  delay(1000);
}
