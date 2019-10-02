/******************************************************************************
Final_Code.ino
Schetch to control the sensors of the instrumented glove designed by Federico Sanna
  (https://www.sparkfun.com/products/10264)
Federico Sanna @ Imperial College London
July 30, 2019

Connect the BNC cable to ground and to the Digital Pin no. 10.

For the flex sensors:
Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 (or other analog inputs) to GND.
- The flex sensor should connect from A0 (or other analog inputs) to 5V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

For the accelertator: 
Connect the //TOBECONTINUED

To save the data from the serial port to a .txt file, create a python code as follow:
                  ##############
                  ## Script listens to serial port and writes contents into a file
                  ##############
                  ## requires pySerial to be installed 
                  import serial
                  
                  serial_port = '/dev/ttyACM0';
                  baud_rate = 9600; #In arduino, Serial.begin(baud_rate)
                  write_to_file_path = "output.txt";
                  
                  output_file = open(write_to_file_path, "w+");
                  ser = serial.Serial(serial_port, baud_rate)
                  while True:
                      line = ser.readline();
                      line = line.decode("utf-8") #ser.readline returns a binary, convert to string
                      print(line);
                      output_file.write(line);
End of python code


Development environment specifics:
Arduino 1.8.3
******************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

const int FLEX_PIN0 = A0; // Pin connected to voltage divider output
const int FLEX_PIN1 = A1; // Pin connected to voltage divider output
const int FLEX_PIN2 = A2; // Pin connected to voltage divider output
const int FLEX_PIN3 = A3; // Pin connected to voltage divider output
//CAREFULL: the pins A4 and A5 are used for the I2C comunication, so you cannot make use of them
const int FLEX_PIN6 = A6; // Pin connected to voltage divider output
const int FLEX_PIN7 = A7; // Pin connected to voltage divider output
const int FLEX_PIN8 = A8; // Pin connected to voltage divider output
const int FLEX_PIN9 = A9; // Pin connected to voltage divider output
const int FLEX_PIN10 = A10; // Pin connected to voltage divider output
const int FLEX_PIN11 = A11; // Pin connected to voltage divider output
const int buttonPin = 2;     // the number of the pushbutton pin
const int squareSignFromArduino = 3;
const int ledPin =  13;      // the number of the LED pin
const int BNC_QuattrocentoPin =  10;      // the number of the pin to signal to the Quattrocento
                                          // that the aquisition is starting


// variables will change:
//volatile int buttonState = 0; // variable for reading the pushbutton status
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 5.08; // Necessary: measured voltage of Ardunio 5V line
const float R_DIV = 46500.0; // Measured resistance of 46.5k resistor. 
                             // Useful but not necessary, as the calibration
                             // code will calculate the resistance automatically
const int index_max_pin = FLEX_PIN11;  // Each pin is indexed with a number.
                                // Max pin here is A11
                                // Max Index should correspond to 65
float STRAIGHT_RESISTANCE[index_max_pin] = {};
float BEND_RESISTANCE[index_max_pin] = {};

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {
  return last_read_time;
}
inline float get_last_x_angle() {
  return last_x_angle;
}
inline float get_last_y_angle() {
  return last_y_angle;
}
inline float get_last_z_angle() {
  return last_z_angle;
}
inline float get_last_gyro_x_angle() {
  return last_gyro_x_angle;
}
inline float get_last_gyro_y_angle() {
  return last_gyro_y_angle;
}
inline float get_last_gyro_z_angle() {
  return last_gyro_z_angle;
}


void setupSensor()
{
  // 1.) Set the accelerometer range. I left the other possible settings that could
  // be used, but in general you will not require them for the kind of application
  // the glove is intended for
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity. We will not really need this
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() {

  Serial.begin(115200);

   // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the BNC pin as an output:
  pinMode(BNC_QuattrocentoPin, OUTPUT);
  // initialize the pin connected to the syncro Arduino as input and enable the internal pull-up resistor:
  pinMode(squareSignFromArduino, INPUT_PULLUP);
  // initialize the pushbutton pin as input:
  pinMode(buttonPin, INPUT);
  // initialize the flex sensors pins as inputs:
  pinMode(FLEX_PIN0, INPUT);
  pinMode(FLEX_PIN1, INPUT);
  pinMode(FLEX_PIN2, INPUT);
  pinMode(FLEX_PIN3, INPUT);
  pinMode(FLEX_PIN6, INPUT);
  pinMode(FLEX_PIN7, INPUT);
  pinMode(FLEX_PIN8, INPUT);
  pinMode(FLEX_PIN9, INPUT);
  pinMode(FLEX_PIN10, INPUT);
  pinMode(FLEX_PIN11, INPUT);
  //initialise default values for flex sensors
  //STRAIGHT_RESISTANCE[0] refers to the resistance of the flex sensor number 0 when at 180° angle
  //STRAIGHT_RESISTANCE[1] refers to the resistance of the flex sensor number 1 when at 180° angle
  for (int i = 0; i < index_max_pin; i++){
    STRAIGHT_RESISTANCE[i] = 33218.44;
    BEND_RESISTANCE[i] = 82426.47;
    }

  while (!Serial) {
    delay(1); // Here not needed because we are using an Arduino Mega, but if using other boards,
    // it will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  // MESSAGE TO THE NEXT IMPLEMENTER:
  // FROM TIME TO TIME THE SENSORS WILL STOP OUTPUTTING MEANINGFUL DATA.
  // IMPORTANT TO WRITE A CALIBRATION CODE TO LAUNCH ONCE EVERY WHILE TO
  // RESET THE SENSORS TO MAKE SURE THEY'RE OUTPUTING MEANINGFUL RESULTS

  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);


}

void loop() {
  // When you receive an input from the Arduino, start saving the data
  int record = digitalRead(squareSignFromArduino);
  double dT;   // to calculate variations of angle and acceleration
  lsm.read();  // ask it to read in the data 

  // Get a new sensor event
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  unsigned long t_now = millis();

  float accel_x = a.acceleration.x;
  float accel_y = a.acceleration.y;
  float accel_z = a.acceleration.z;
  float gyro_x = g.gyro.x;
  float gyro_y = g.gyro.y;
  float gyro_z = g.gyro.z;

  float angle_0 = CalculateAngleAtPin(FLEX_PIN0);
  float angle_1 = CalculateAngleAtPin(FLEX_PIN1);
  float angle_2 = CalculateAngleAtPin(FLEX_PIN2);
  float angle_3 = CalculateAngleAtPin(FLEX_PIN3);
  float angle_6 = CalculateAngleAtPin(FLEX_PIN6);
  float angle_7 = CalculateAngleAtPin(FLEX_PIN7);
  float angle_8 = CalculateAngleAtPin(FLEX_PIN8);
  float angle_9 = CalculateAngleAtPin(FLEX_PIN9);
  float angle_10 = CalculateAngleAtPin(FLEX_PIN10);
  float angle_11 = CalculateAngleAtPin(FLEX_PIN11);

  /*Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");*/

  /*Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");*/

  /*Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
    Serial.println("");*/

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt = (t_now - get_last_time()) / 1000.0;
  float gyro_angle_x = gyro_x * dt + get_last_x_angle();
  float gyro_angle_y = gyro_y * dt + get_last_y_angle();
  float gyro_angle_z = gyro_z * dt + get_last_z_angle();

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
  float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle


  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

  // Send the data to the serial port
  /*Serial.print(F("DEL:"));              //Delta T
    Serial.print(dt, DEC);
    Serial.print(F("#ACC:"));              //Accelerometer angle
    Serial.print(accel_angle_x, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_y, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_z, 2);
    Serial.print(F("#GYR:"));
    Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_y, 2);
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_z, 2);*/

  // Following code get executed when receiving the square wave
  if (record == HIGH) {
    // FOR FIRMAN: THIS IS THE PART THAT SHOULD BE SAVED, EVERYTHING PRINTED IN HERE
    Serial.print(F("Angle with respect to the horizontal:"));      //Filtered angle
    Serial.print(angle_x, 2);
    Serial.print(F(","));
    Serial.print(angle_y, 2);
    Serial.print(F(","));
    Serial.print(angle_z, 2);
    Serial.println(F(""));
  
    Serial.println("Flex sensor 0, bend: " + String(angle_0) + " degrees");
    Serial.println("Flex sensor 1, bend: " + String(angle_1) + " degrees");
    Serial.println("Flex sensor 2, bend: " + String(angle_2) + " degrees");
    Serial.println("Flex sensor 3, bend: " + String(angle_3) + " degrees");
    Serial.println("Flex sensor 6, bend: " + String(angle_6) + " degrees");
    Serial.println("Flex sensor 7, bend: " + String(angle_7) + " degrees");
    Serial.println("Flex sensor 8, bend: " + String(angle_8) + " degrees");
    Serial.println("Flex sensor 9, bend: " + String(angle_9) + " degrees");
    Serial.println("Flex sensor 10, bend: " + String(angle_10) + " degrees");  
    Serial.println("Flex sensor 11, bend: " + String(angle_11) + " degrees");
  
    Serial.println();
  
    Serial.println("The time now is: " + String(millis()) + " ms");
  }

  
  //The following code is to check for when calibration should be performed
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      // do nothing, but can be implemented to add functions for multiple consequent presses
    } else {
      // if the current state is LOW then the button went from on to off:
      Calibration();  //perform calibration on release of the button
      
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  // Delay so we don't swamp the serial port
  //delay(5);
}


float CalculateAngleAtPin (int NumberOfPin){
  int flexADC = analogRead(NumberOfPin);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  float angle = map(flexR, STRAIGHT_RESISTANCE[NumberOfPin], BEND_RESISTANCE[NumberOfPin],
                   0, 90.0);
  return angle;
  }

void Calibration() {
  // Blink just before recording values for straight hand
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(ledPin, LOW);    
  delay(200); 
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(ledPin, LOW);    
  delay(200); 

  // Reset all the values of the accelerometer and gyroscope
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);

  // Leave LED on during aquisition of resistance
  digitalWrite(ledPin, HIGH);  
  // Calculate resistance when hand is straight
  int flexADC_0 = analogRead(FLEX_PIN0);
  int flexADC_1 = analogRead(FLEX_PIN1);
  int flexADC_2 = analogRead(FLEX_PIN2);
  int flexADC_3 = analogRead(FLEX_PIN3);
  int flexADC_6 = analogRead(FLEX_PIN6);
  int flexADC_7 = analogRead(FLEX_PIN7);
  int flexADC_8 = analogRead(FLEX_PIN8);
  int flexADC_9 = analogRead(FLEX_PIN9);
  int flexADC_10 = analogRead(FLEX_PIN10);
  int flexADC_11 = analogRead(FLEX_PIN11);
  float flexV_0 = flexADC_0 * VCC / 1023.0;
  float flexV_1 = flexADC_1 * VCC / 1023.0;
  float flexV_2 = flexADC_2 * VCC / 1023.0;
  float flexV_3 = flexADC_3 * VCC / 1023.0;
  float flexV_6 = flexADC_6 * VCC / 1023.0;
  float flexV_7 = flexADC_7 * VCC / 1023.0;
  float flexV_8 = flexADC_8 * VCC / 1023.0;
  float flexV_9 = flexADC_9 * VCC / 1023.0;
  float flexV_10 = flexADC_10 * VCC / 1023.0;
  float flexV_11 = flexADC_11 * VCC / 1023.0;
  float flexR_0 = R_DIV * (VCC / flexV_0 - 1.0);
  float flexR_1 = R_DIV * (VCC / flexV_1 - 1.0);
  float flexR_2 = R_DIV * (VCC / flexV_2 - 1.0);
  float flexR_3 = R_DIV * (VCC / flexV_3 - 1.0);
  float flexR_6 = R_DIV * (VCC / flexV_6 - 1.0);
  float flexR_7 = R_DIV * (VCC / flexV_7 - 1.0);
  float flexR_8 = R_DIV * (VCC / flexV_8 - 1.0);
  float flexR_9 = R_DIV * (VCC / flexV_9 - 1.0);
  float flexR_10 = R_DIV * (VCC / flexV_10 - 1.0);
  float flexR_11 = R_DIV * (VCC / flexV_11 - 1.0);
  STRAIGHT_RESISTANCE[FLEX_PIN0] = flexR_0;
  STRAIGHT_RESISTANCE[FLEX_PIN1] = flexR_1;
  STRAIGHT_RESISTANCE[FLEX_PIN2] = flexR_2;
  STRAIGHT_RESISTANCE[FLEX_PIN3] = flexR_3;
  STRAIGHT_RESISTANCE[FLEX_PIN6] = flexR_6;
  STRAIGHT_RESISTANCE[FLEX_PIN7] = flexR_7;
  STRAIGHT_RESISTANCE[FLEX_PIN8] = flexR_8;
  STRAIGHT_RESISTANCE[FLEX_PIN9] = flexR_9;
  STRAIGHT_RESISTANCE[FLEX_PIN10] = flexR_10;
  STRAIGHT_RESISTANCE[FLEX_PIN11] = flexR_11;
  digitalWrite(ledPin, LOW);  

  // Wait 3 seconds to allow for patient to bend fingers
  for (int i = 0; i < 10; i++){
    digitalWrite(ledPin, HIGH);   
    delay(100);             
    digitalWrite(ledPin, LOW);    
    delay(100);
    }
  delay(500);

  // Signal that acquisition is starting by blinking
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(ledPin, LOW);    
  delay(200); 
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(ledPin, LOW);    
  delay(200); 

  // Leave LED on during aquisition of resistance
  digitalWrite(ledPin, HIGH);  
  // Calculate resistance when hand is bent
  flexADC_0 = analogRead(FLEX_PIN0);
  flexADC_1 = analogRead(FLEX_PIN1);
  flexADC_2 = analogRead(FLEX_PIN2);
  flexADC_3 = analogRead(FLEX_PIN3);
  flexADC_6 = analogRead(FLEX_PIN6);
  flexADC_7 = analogRead(FLEX_PIN7);
  flexADC_8 = analogRead(FLEX_PIN8);
  flexADC_9 = analogRead(FLEX_PIN9);
  flexADC_10 = analogRead(FLEX_PIN10);
  flexADC_11 = analogRead(FLEX_PIN11);
  flexV_0 = flexADC_0 * VCC / 1023.0;
  flexV_1 = flexADC_1 * VCC / 1023.0;
  flexV_2 = flexADC_2 * VCC / 1023.0;
  flexV_3 = flexADC_3 * VCC / 1023.0;
  flexV_6 = flexADC_6 * VCC / 1023.0;
  flexV_7 = flexADC_7 * VCC / 1023.0;
  flexV_8 = flexADC_8 * VCC / 1023.0;
  flexV_9 = flexADC_9 * VCC / 1023.0;
  flexV_10 = flexADC_10 * VCC / 1023.0;
  flexV_11 = flexADC_11 * VCC / 1023.0;
  flexR_0 = R_DIV * (VCC / flexV_0 - 1.0);
  flexR_1 = R_DIV * (VCC / flexV_1 - 1.0);
  flexR_2 = R_DIV * (VCC / flexV_2 - 1.0);
  flexR_3 = R_DIV * (VCC / flexV_3 - 1.0);
  flexR_6 = R_DIV * (VCC / flexV_6 - 1.0);
  flexR_7 = R_DIV * (VCC / flexV_7 - 1.0);
  flexR_8 = R_DIV * (VCC / flexV_8 - 1.0);
  flexR_9 = R_DIV * (VCC / flexV_9 - 1.0);
  flexR_10 = R_DIV * (VCC / flexV_10 - 1.0);
  flexR_11 = R_DIV * (VCC / flexV_11 - 1.0);
  BEND_RESISTANCE[FLEX_PIN0] = flexR_0;
  BEND_RESISTANCE[FLEX_PIN1] = flexR_1;
  BEND_RESISTANCE[FLEX_PIN2] = flexR_2;
  BEND_RESISTANCE[FLEX_PIN3] = flexR_3;
  BEND_RESISTANCE[FLEX_PIN6] = flexR_6;
  BEND_RESISTANCE[FLEX_PIN7] = flexR_7;
  BEND_RESISTANCE[FLEX_PIN8] = flexR_8;
  BEND_RESISTANCE[FLEX_PIN9] = flexR_9;
  BEND_RESISTANCE[FLEX_PIN10] = flexR_10;
  BEND_RESISTANCE[FLEX_PIN11] = flexR_11;  
  digitalWrite(ledPin, LOW); 

  // Send a signal to the Quatrocento that we are starting recording now by pulling a pin high
  digitalWrite(BNC_QuattrocentoPin, HIGH);
  Serial.println("Raising edge sent to Quattrocento now. Time is" + String(millis()) + " ms");
  delay(200);             
  digitalWrite(BNC_QuattrocentoPin, LOW); 
}
