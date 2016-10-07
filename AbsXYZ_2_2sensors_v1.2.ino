#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bnoA = Adafruit_BNO055(55, BNO055_ADDRESS_A);
Adafruit_BNO055 bnoB = Adafruit_BNO055(55, BNO055_ADDRESS_B);

String inputString = "";
int mode = 4;
int mode_prev = 0;
//int SENSOR_RESET = 8;
int SENSOR_A_ADDR = 2;
int SENSOR_B_ADDR = 3;
int SENSOR_A_RST = 4;
int SENSOR_B_RST = 5;

void setup(void)
{
  pinMode(SENSOR_A_ADDR, OUTPUT);
  pinMode(SENSOR_B_ADDR, OUTPUT);
  pinMode(SENSOR_A_RST, OUTPUT);
  pinMode(SENSOR_B_RST, OUTPUT);
  digitalWrite(SENSOR_A_ADDR, LOW);
  digitalWrite(SENSOR_B_ADDR, HIGH);

  //RESET SENSOR A:
  digitalWrite(SENSOR_A_RST, LOW);
  delay(100);
  digitalWrite(SENSOR_A_RST, HIGH);

  //RESET SENSOR B:
  digitalWrite(SENSOR_B_RST, LOW);
  delay(100);
  digitalWrite(SENSOR_B_RST, HIGH);
  //mode_prev = mode;

  Serial.begin(9600);
  inputString.reserve(200);
  //pinMode(SENSOR_RESET, OUTPUT);
  //digitalWrite(SENSOR_RESET, HIGH);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bnoA.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055_A detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bnoA.setExtCrystalUse(true);

  if (!bnoB.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055_B detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bnoB.setExtCrystalUse(true);
}
void loop(void)
{ //mode 1 --------- Euler
  //mode 2 ---------- Gravity vector
  /* Get a new sensor event */
  if (mode == 1) {
    /* Get a new sensor event */
    sensors_event_t event;
    bnoA.getEvent(&event);
    /* Display the floating point data */
    Serial.print("Mode ");
    Serial.println(mode);
    Serial.print("[ ");
    Serial.print(event.orientation.x, 4);
    Serial.print(" ");
    Serial.print(event.orientation.y, 4);
    Serial.print(" ");
    Serial.print(event.orientation.z, 4);
    Serial.println(" ]");
    delay(10);

  }

  else if (mode == 2) { //GRAVITY DATA of sensor A
    if (mode_prev != 2) {
      //Switch addresses:
      /*digitalWrite(SENSOR_A_ADDR, LOW);
      digitalWrite(SENSOR_B_ADDR, HIGH);

      //RESET SENSOR A:
      digitalWrite(SENSOR_A_RST, LOW);
      delay(100);
      digitalWrite(SENSOR_A_RST, HIGH);

      //RESET SENSOR B:
      digitalWrite(SENSOR_B_RST, LOW);
      delay(100);
      digitalWrite(SENSOR_B_RST, HIGH);
      mode_prev = mode;

      while (!bno.begin())
      {
        //keep trying to conect
      }
      delay(100);
      bno.setExtCrystalUse(true);*/
    }
    sensors_event_t event;
    imu::Vector<3> grav = bnoA.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("Mode ");
    Serial.println(mode);
    Serial.print("[ ");
    Serial.print(grav.x());
    Serial.print(" ");
    Serial.print(grav.y());
    Serial.print(" ");
    Serial.print(grav.z());
    Serial.println(" ]");
    delay(10);
  }
  else if (mode == 3) { //GRAVITY DATA of sensor B
  
    if (mode_prev != 3) {
      //Switch addresses:
      /*
      digitalWrite(SENSOR_B_ADDR, LOW);
      digitalWrite(SENSOR_A_ADDR, HIGH);

      //RESET SENSOR B:
      digitalWrite(SENSOR_B_RST, LOW);
      delay(100);
      digitalWrite(SENSOR_B_RST, HIGH);

      //RESET SENSOR A:
      digitalWrite(SENSOR_A_RST, LOW);
      delay(100);
      digitalWrite(SENSOR_A_RST, HIGH);
      mode_prev = mode;

      while (!bno.begin())
      {
        //keep trying to conect
      }
      delay(100);
      bno.setExtCrystalUse(true);*/
    }
    sensors_event_t event;
    imu::Vector<3> grav = bnoB.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("Mode ");
    Serial.println(mode);
    Serial.print("[ ");
    Serial.print(grav.x());
    Serial.print(" ");
    Serial.print(grav.y());
    Serial.print(" ");
    Serial.print(grav.z());
    Serial.println(" ]");
    delay(10);

  }
    else if(mode==4){ //Prints data from both sensors
      sensors_event_t event;
    
    /*Serial.print("Mode ");
    Serial.println(mode);*/
    imu::Vector<3> gravA = bnoA.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    imu::Vector<3> gravB = bnoB.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("[ ");
    Serial.print(gravA.x());
    Serial.print(" ");
    Serial.print(gravA.y());
    Serial.print(" ");
    Serial.print(gravA.z());
    Serial.print(" ]   ");
    //delay(10);
    
    Serial.print("{ ");
    Serial.print(gravB.x());
    Serial.print(" ");
    Serial.print(gravB.y());
    Serial.print(" ");
    Serial.print(gravB.z());
    Serial.println(" }");
    
    }

}

void serialEvent() {
  boolean stringComplete = false;
  inputString = "";
  while (Serial.available()) {
    // get the new byte:
    while (!stringComplete) {
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n') {
        stringComplete = true;
      }
    }

    int new_mode = inputString.toInt();
    Serial.print("New Mode:");
    Serial.println(new_mode);
    if ((new_mode == 1) || (new_mode == 2) || (new_mode == 3)||(new_mode == 4)) {
      mode = new_mode;
    }


  }
}

void reset_sensor()
{
  if (!bnoA.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bnoA.setExtCrystalUse(true);
}


