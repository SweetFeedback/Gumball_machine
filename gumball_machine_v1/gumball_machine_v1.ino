int inByte = 0;         


// These constants won't change.  They're used to give names
// to the pins used:

// Sensors' pin
const int MicInPin = A0;  // Analog input pin that the microphone is attached to
const int PhotodlnInPin = A1;  // Analog input pin that the photodarlington is attached to
const int ThermtrInPin = A2;  // Analog input pin that the thermistor is attached to
const int DistanceInPin = A3; // Analog input pin that the distance sensor is attached to
const int WindowInPin = 2;
//Actuators' pin
const int SpeakerOutPin = 8; // Speaker connected from digital pin 8 to ground
const int LedOutPin = 12; // LED connected from digital pin 12 to ground
const int MotorOutPin = 13; // Motor connected from digital pin 13 to ground

const int sensorNum = 4;
int sensorValue[sensorNum] = {0};        // initialize value read from the pot
int outputValue[sensorNum] = {0};        // initialize value output to the PWM (analog out)
int toShow[sensorNum] = {0,0,0,0};       // 0 if the data is not going to show, 1 otherwise

const int MAX_DISTANCE = 1000;
const int MAX_NOISE_LEVEL = 10;
const int NOISE_THRESHOLD = 15;
int noiseLevel = 0;
int ledState = LOW;

boolean humanState = false;
int smoothedDistance = 0;
int windowState;             // the current reading from the input pin
int lastWindowState = LOW;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time; increase if the output flickers


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  pinMode(LedOutPin, OUTPUT);  
  pinMode(SpeakerOutPin, OUTPUT);  
  pinMode(MotorOutPin, OUTPUT);
  pinMode(WindowInPin, INPUT);
  digitalWrite(MotorOutPin, LOW);
  //  establishContact(); 

}

void loop() {
  // read the analog in value:
  sensorValue[0] = analogRead(MicInPin);
  sensorValue[1] = analogRead(PhotodlnInPin);  
  sensorValue[2] = analogRead(ThermtrInPin);  
  sensorValue[3] = analogRead(DistanceInPin);  
  
  // Sensor calibration
  outputValue[0] = map(sensorValue[0],  500, 700, 0, 255);  
  outputValue[1] = map(sensorValue[1],  0, 1023, 0, 255);  
  outputValue[2] = map(sensorValue[2],  50, 150, 0, 255);  
  outputValue[3] = distanceCalibration(sensorValue[3]);
//\\  windowUpdate();
  smoothedDistance = lowpassFilter(outputValue[3], smoothedDistance, 0.25);
/*  
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
  printData();

*/
  /*
  // for processing
   Serial.write(outputValue[0]);
   Serial.write(outputValue[1]);
   Serial.write(outputValue[2]);
   */

  /*
  // for php
  Serial.print(outputValue[0]);
  Serial.print(",");
  Serial.print(outputValue[1]);
  Serial.print(",");
  Serial.print(outputValue[2]);
  Serial.println("");*/
  checkWhetherToGiveCandy();
  

  
  delay(100);    
 
}

void windowUpdate(){
  int reading = digitalRead(WindowInPin);
  if (reading != lastWindowState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  if ((millis() - lastDebounceTime) > debounceDelay) {
    windowState = reading;
  }
  Serial.print("window: ");
  Serial.println(reading);
  lastWindowState = reading;
}

void checkWhetherToGiveCandy(){
    if(Serial.available( ) > 0) {
      inByte = Serial.read();
      if (inByte == 'A') {
        giveCandies();
      }
    }
}
void giveCandies(){
    Serial.print("Candies come!");
    digitalWrite(MotorOutPin, LOW);
    digitalWrite(MotorOutPin, HIGH);   // candy coming
    delay(2000);
    digitalWrite(MotorOutPin, LOW);
    delay(500);
}

// For Human-friendly reading. For developer to debug
void printData(){
  int showCount = 0;
  for(int i = 0; i < sensorNum; i ++){
    if(toShow[i] == 1){
        if(showCount>0)
          Serial.print(", ");
        Serial.print(outputValue[i]);

      showCount++;
    }
  }
  Serial.print("\n");
}

// Note: Distance sensor can measure from 20 cm to 150 cm
int distanceCalibration(int input){
  // Valid raw data range is from 80 ~ 490
  if(input >= 80 && input <= 490)
    return 9462/(input - 16.92);
  return MAX_DISTANCE;

}
boolean isNoisy(){
  return (noiseLevel == MAX_NOISE_LEVEL);
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
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}
int lowpassFilter(int newValue, int oldValue, float alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}
