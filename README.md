# PoolWatch
read PH sensor values with an Arduino MKR 1310 and send the data to LoRa

## PH Sensor

## Temperature Sensor
- DS18B20 (not DS18S20!) one wire https://github.com/milesburton/Arduino-Temperature-Control-Library
- SHT-30 I2C Sensirion SHT-3
- AHT20 I2C Temp/RH sensor
- 10K precision epoxy thermistor 3950 NTC for internal temperature/humidity measure

## Battery and Solar
Inputs: 
Li-Po 3.7V, VIN 5-6V
- LiPo
  Operating Temperature: -10 to 50 °C
  Charging Temperature: -10 to 50 °C
  Dont charge in cold temperature!!
  use minimum 1024 mAh
  
- NiMH
  Operating Temperature: -15.5 to 50 °C
  Charging Temperature: -20 to 50 °C
  IKEA AA HR6 1.2V 2450mAh

VIN - R1 --- R2 - GND
          |
	  A1

VA1 = VIN * (R2/(R1+R2))
Vref = 1.1 V  ??

Resolution: 12-bit -> 0...1023 -> 3.3 V / 1024 = 3.22 mV
bat = analogRead(A1);
vbat = bat * 3.3/1023 * ((R1+R2)/R2); 

## Low Power
https://www.youtube.com/watch?v=urLSDi7SD8M

## LoRa
### Antenna
Original Arduino MKR 1310 claims 2dB gain. Probably not suitable for mounting.
Alternatives:


## TTN / Swisscom
### TTN
Applications -> Add application
- Frequency: Europe 868-870 MHz (SF9 for RX2), 434 MHz
- LoRaWAN version: MAC V1.0.2
- Regional Parameter version: PHY V1.0.2 REV A
- Networ Server Address: eu1.cloud.thethings.network
- Application Server Address: eu1.cloud.thethings.network
- Join Server Address: eu1.cloud.thethings.network

## Cloud
- InfluxDB Cloud 2.0
- Azure IoT		10.-/Mt
- AWS IoT Core	

## Local Server
### Dynamic DNS
- https://www.dynu.com
- https://www.cloudflare.com
- https://www.noip.com
Swisscom

### NodeRED

### InfluxDB

### Grafana

### Notification



