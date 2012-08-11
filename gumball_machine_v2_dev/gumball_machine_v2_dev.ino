// These constants won't change.  They're used to give names
// to the pins used:

//#include "dev.h"

// Sensors' pin
const int MicInPin = A0;  // Analog input pin that the microphone is attached to
const int PhotodlnInPin = A1;  // Analog input pin that the photodarlington is attached to
const int ThermtrInPin = A2;  // Analog input pin that the thermistor is attached to
const int DistanceInPin = A3; // Analog input pin that the distance sensor is attached to
const int WindowInPin = 2;
const int BottomSwitchInPin = 3;

//Actuators' pin
const int SpeakerOutPin = 8; // Speaker connected from digital pin 8 to ground
const int LedOutPin = 12; // LED connected from digital pin 12 to ground
const int MotorOutPin = 13; // Motor connected from digital pin 13 to ground



const int sensorNum = 6;
int sensorValue[sensorNum] = {0};        // initialize value read from the pot
int outputValue[sensorNum] = {0};        // initialize value output to the PWM (analog out)
int print_mask[sensorNum] = {0};       // 0 if the data is not going to show, 1 otherwise


const int MAX_DISTANCE = 1000;
const int MAX_NOISE_LEVEL = 10;
const int NOISE_THRESHOLD = 15;


int lastSwitchState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

int noiseLevel = 0;
int ledState = LOW;

boolean humanState = false;
int smoothedDistance = 0;

enum tempUnit{Kelvin,Celcius,Fahrenheit};

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  pinMode(LedOutPin, OUTPUT);  
  pinMode(SpeakerOutPin, OUTPUT);  
  pinMode(MotorOutPin, OUTPUT);
  pinMode(WindowInPin, INPUT);
  pinMode(BottomSwitchInPin, INPUT);
  digitalWrite(MotorOutPin, LOW);
}

void loop() {
  // read the analog in value:
  sensorValue[0] = analogRead(MicInPin);
  sensorValue[1] = analogRead(PhotodlnInPin);  
  sensorValue[2] = analogRead(ThermtrInPin);  
  sensorValue[3] = analogRead(DistanceInPin);
  sensorValue[4] = getWindowState();
  sensorValue[5] = getSwtichState();
  
  // Sensor calibration
  outputValue[0] = map(sensorValue[0],  500, 700, 0, 255);  
  outputValue[1] = map(sensorValue[1],  0, 1023, 0, 255);  
  outputValue[2] = thermistorCalibration(sensorValue[2], Celcius);  
  outputValue[3] = distanceCalibration(sensorValue[3]);
  outputValue[4] = sensorValue[4];
  
  simple_led_task();
  
  // for php
  sendToServer();
  delay(100);
}

void simple_led_task(){
  smoothedDistance = lowpassFilter(outputValue[3], smoothedDistance, 0.25); 
  if(isHumanAround(smoothedDistance)){
    ledControl(HIGH);
    humanState = true;
  }
  else{
    if(humanState == true)
      playDisappointedSound();
    ledControl(LOW);
    humanState = false;
  }
}
int getSwtichState(){
  int reading = digitalRead(BottomSwitchInPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (reading != lastSwitchState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
 
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    switchState = reading;
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastSwitchState:
  lastSwitchState = reading;
  return switchState;
}
int getWindowState(){
  int reading = digitalRead(WindowInPin);
  if(reading == HIGH) return LOW;
  return HIGH;
}

void giveCandies(){
  digitalWrite(MotorOutPin, LOW);
  digitalWrite(MotorOutPin, HIGH);   // candy coming
  delay(500);
  digitalWrite(MotorOutPin, LOW);
  delay(500);
}

void sendToServer(){
  if(Serial.available() > 0) {
    int inByte = Serial.read();
    if (inByte == 'A') {
      printData();
      Serial.println();
    }else if (inByte == 'B') {
        giveCandies();
    }
  }
}

// For Human-friendly reading. For developer to debug
void printData(){
  int showCount = 0;
  for(int i = 0; i < sensorNum; i ++){
    if(print_mask[i] == 1){
      if(showCount>0)
        Serial.print(", ");
      Serial.print(outputValue[i]);
      showCount++;
    }
  }
  Serial.println();
}

// This is the calibration function for Sharp Distance Sensor 2Y0A02
// Note: this distance sensor can measure from 20 cm to 150 cm
int distanceCalibration(int input){
  // Valid raw data range is from 80 ~ 490
  if(input >= 80 && input <= 490)
    return 9462/(input - 16.92);
  return MAX_DISTANCE;
}

// This is the calibration function for thermistor P/N:NTCLE413E2103H400
// parameter RawADC is the analogReading from the thermistor
// parameter Unit is the temperature unit of the return value
double thermistorCalibration(int RawADC, int Unit) {
 long double temp;
 long double A,B,C,D;

// this is the coefficient for the thermistor P/N:NTCLE413E2103H400
  A = 0.0012;
  B = 2.2614e-004;
  C = 7.0822e-007;
  D = 6.7885e-008;
  double R = 1000;
  double RT = (1024*R/RawADC) - R;
  
// Steinhart–Hart equation
// {1 \over T} = A + B \ln(R) + C (\ln(R))^3 \, 
// check wiki for more info http://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
   temp = log(RT);
   long double divisor = (A + (B + (C + D * temp)* temp )* temp);
   temp = 1/ divisor;
  if(Unit == Kelvin)
    return temp;
  else if(Unit == Celcius)
    return temp = temp - 273.15;            // Convert Kelvin to Celcius
  else if(Unit == Fahrenheit)
    return temp = (temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  
}

boolean isHumanAround(int distance){
  return (distance < 500);
}

void ledControl(int input){
   if(input!= ledState){
      digitalWrite(LedOutPin, input);
      ledState = input;
   }
}

void handleNoiseLevel(){
  if( abs(outputValue[0] - 150) > NOISE_THRESHOLD){
    if(noiseLevel < MAX_NOISE_LEVEL)
      noiseLevel++;
  }
  
  else if(noiseLevel > 0){
    noiseLevel--;
  } 
}

void playDisappointedSound() {
  for (int i = 0; i <= 60; i++) {
    tone(SpeakerOutPin, 880 * pow(2, -i / 60.0));
    delay(5);
    noTone(SpeakerOutPin);
  }
}

int lowpassFilter(int newValue, int oldValue, float alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}
