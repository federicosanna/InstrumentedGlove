/******************************************************************************
FlexSensor.ino
Schetch to control the flex sensor of the instrumented glove designed by Federico Sanna
  (https://www.sparkfun.com/products/10264)
Federico Sanna @ Imperial College London
July 16, 2018

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 (or other analog inputs) to GND.
- The flex sensor should connect from A0 (or other analog inputs) to 5V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.8.3
******************************************************************************/
const int FLEX_PIN0 = A0; // Pin connected to voltage divider output
const int FLEX_PIN3 = A3; // Pin connected to voltage divider output
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
//volatile int buttonState = 0; // variable for reading the pushbutton status
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 5.08; // Measured voltage of Ardunio 5V line
const float R_DIV = 46500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
//float STRAIGHT_RESISTANCE = 33218.44; // resistance when straight
//float BEND_RESISTANCE = 82426.47; // resistance at 90 deg
float STRAIGHT_RESISTANCE[30] = {};
float BEND_RESISTANCE[30] = {};

void setup() 
{
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  // Attach an interrupt to the ISR vector
  //attachInterrupt(0, Calibration, FALLING);
  // initialize the flex sensors pins as inputs:
  pinMode(FLEX_PIN0, INPUT);
  pinMode(FLEX_PIN3, INPUT);
  //initialise default values for flex sensors
  for (int i = 0; i < 30; i++){
    STRAIGHT_RESISTANCE[i] = 33218.44;
    BEND_RESISTANCE[i] = 82426.47;
    }
}

void loop() 
{
  float angle_0 = CalculateAngleAtPin(FLEX_PIN0);
  float angle_3 = CalculateAngleAtPin(FLEX_PIN3);
  Serial.println("Bend: " + String(angle_0) + " degrees");
  Serial.println("Bend: " + String(angle_3) + " degrees");
  Serial.println();


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
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  
  delay(500);
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
  digitalWrite(13, LOW);    
  delay(200); 
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(13, LOW);    
  delay(200); 

  // Leave LED on during aquisition of resistance
  digitalWrite(ledPin, HIGH);  
  // Calculate resistance when hand is straight
  int flexADC_0 = analogRead(FLEX_PIN0);
  int flexADC_3 = analogRead(FLEX_PIN3);
  float flexV_0 = flexADC_0 * VCC / 1023.0;
  float flexV_3 = flexADC_3 * VCC / 1023.0;
  float flexR_0 = R_DIV * (VCC / flexV_0 - 1.0);
  float flexR_3 = R_DIV * (VCC / flexV_3 - 1.0);
  STRAIGHT_RESISTANCE[FLEX_PIN0] = flexR_0;
  STRAIGHT_RESISTANCE[FLEX_PIN3] = flexR_3;
  digitalWrite(ledPin, LOW);  

  // Wait 3 seconds to allow for patient to bend fingers
  for (int i = 0; i < 10; i++){
    digitalWrite(ledPin, HIGH);   
    delay(100);             
    digitalWrite(13, LOW);    
    delay(100);
    }
  delay(500);

  // Signal that acquisition is starting by blinking
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(13, LOW);    
  delay(200); 
  digitalWrite(ledPin, HIGH);   
  delay(200);             
  digitalWrite(13, LOW);    
  delay(200); 

  // Leave LED on during aquisition of resistance
  digitalWrite(ledPin, HIGH);  
  // Calculate resistance when hand is straight
  flexADC_0 = analogRead(FLEX_PIN0);
  flexADC_3 = analogRead(FLEX_PIN3);
  flexV_0 = flexADC_0 * VCC / 1023.0;
  flexV_3 = flexADC_3 * VCC / 1023.0;
  flexR_0 = R_DIV * (VCC / flexV_0 - 1.0);
  flexR_3 = R_DIV * (VCC / flexV_3 - 1.0);
  BEND_RESISTANCE[FLEX_PIN0] = flexR_0;
  BEND_RESISTANCE[FLEX_PIN3] = flexR_3;
  digitalWrite(ledPin, LOW); 
}
