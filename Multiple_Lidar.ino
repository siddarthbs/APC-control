#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;

void setup()
{

  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  Wire.begin();


  Serial.begin (9600);

  digitalWrite(4, HIGH);
  sensor.init(true);
  sensor.setTimeout(500);
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  sensor.setMeasurementTimingBudget(100000);
  sensor.setAddress((uint8_t)22);

  digitalWrite(5, HIGH);
  sensor2.init(true);
  sensor2.setTimeout(500);
  sensor2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  sensor2.setMeasurementTimingBudget(100000);

  sensor2.setAddress((uint8_t)25);

  sensor.startContinuous();
  sensor2.startContinuous();



}

void loop()
{

  Serial.print("< ");
  Serial.print(float(sensor.readRangeContinuousMillimeters()));
  //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print(" ");

  Serial.print(float(sensor2.readRangeContinuousMillimeters()));
  Serial.print(" >");
  //if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  //delay(100);
  Serial.println();
}
