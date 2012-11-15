// These constants won't change.  They're used to give names
// to the pins used:

// Sensors' pin
const int MicInPin = A0;  // Analog input pin that the microphone is attached to
const int PhotodlnInPin = A1;  // Analog input pin that the photodarlington is attached to
const int ThermtrInPin = A2;  // Analog input pin that the thermistor is attached to
const int DistanceInPin = A3; // Analog input pin that the distance sensor is attached to
const int WindowInPin = 2;
const int BottomSwitchInPin = 3;
//Actuators' pin
const int MotorOutPin = 4; // Motor connected from digital pin 13 to ground
const int SpeakerOutPin = 8; // Speaker connected from digital pin 8 to ground
const int LedOutPin = 10; // LED connected from digital pin 12 to ground

const int sensorNum = 5;
int sensorValue[sensorNum] = {0};        // initialize value read from the pot
int outputValue[sensorNum] = {0};        // initialize value output to the PWM (analog out)
int print_mask[sensorNum] = {0};       // 0 if the data is not going to show, 1 otherwise

const int MachineID = 100;
const int MAX_DISTANCE = 1000;
int ledState = LOW;


boolean humanState = false;
int smoothedDistance = 0;
int balanceMicVal = -1;
enum tempUnit{Kelvin,Celcius,Fahrenheit};

const int numReadings = 500;
int micReadings[numReadings] = {0};
long total = 0;
int index = 0;
float outputNoiseLevel = -1;

//for group collaboration experiment
int count = 0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(LedOutPin, OUTPUT);
  pinMode(SpeakerOutPin, OUTPUT);
  pinMode(MotorOutPin, OUTPUT);

  digitalWrite(MotorOutPin, HIGH);
  pinMode(WindowInPin, INPUT);
  pinMode(BottomSwitchInPin, INPUT);
  //digitalWrite(MotorOutPin, LOW);
  flashLed(LedOutPin, 3, 500);
  //playDisappointedSound();
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop() {
  getSensorData();
  serialCallResponse();
  breathingLed();
  //simple_led_task();
}

// Main Loop Tasks
void getSensorData() {
  // read the analog in value:
  sensorValue[0] = analogRead(MicInPin);
  sensorValue[1] = analogRead(PhotodlnInPin);  
  sensorValue[2] = analogRead(ThermtrInPin);  
  sensorValue[3] = analogRead(DistanceInPin);
  sensorValue[4] = getWindowState();
  // sensorValue[5] = digitalRead(BottomSwitchInPin);

  // Sensor calibration

  float newNoiseLevel = noiseLevel(sensorValue[0]);
  if(outputNoiseLevel == -1){
    outputNoiseLevel = newNoiseLevel;
  }else{
    outputNoiseLevel = lowpassFilter(newNoiseLevel,outputNoiseLevel,0.25);
  }
  outputValue[0] = outputNoiseLevel;
  outputValue[1] = map(sensorValue[1],  0, 1023, 0, 255);  
  outputValue[2] = thermistorCalibration(sensorValue[2], Celcius);  
  outputValue[3] = distanceCalibration(sensorValue[3]);
  outputValue[4] = sensorValue[4];
  // outputValue[5] = sensorValue[5];
}

void calcBalanceMicVal(int micVal){
  if(balanceMicVal == -1){
      balanceMicVal = micVal;
      for(int i = 0; i < numReadings; i++){
        micReadings[i] = micVal;
        total += micVal;
      }
  }
  else{
    // subtract the last reading:
    total= total - micReadings[index];         
    // read from the sensor:  
    micReadings[index] =  micVal; 
    // add the reading to the total:
    total= total + micReadings[index];       
    // advance to the next position in the array:  
    index = index + 1;                    
    balanceMicVal = total/numReadings; 
    // if we're at the end of the array...
    if (index >= numReadings)              
      // ...wrap around to the beginning: 
      index = 0;             
  }
}
void serialCallResponse(){
  if(Serial.available() > 0) {
    int inByte = Serial.read();
    int i;
    if (inByte == 'A') {
        giveCandies();
    }else if (inByte == 'B') {
        for(i = 0; i < sensorNum -1; i++){
            Serial.print(outputValue[i]);
              Serial.print(",");  
        }
        Serial.println(outputValue[i]);
    }else if (inByte == 'C'){
        playDisappointedSound();
    }else if (inByte == 'D'){ //incremental positive feedback
        count += 1;
        if(count >= 10){
          for (int i = 0; i <= 80; i++) {
            tone(SpeakerOutPin, (400+count*40) * pow(1+i/60.0, 4));
            delay(5);
            noTone(SpeakerOutPin);
          }
          count = 0;
        }else{
          for (int i = 0; i <= 30; i+=1) {
            tone(SpeakerOutPin, (400+count*40) * pow(1+i/60.0, 2));
            delay(7);
            noTone(SpeakerOutPin);
          }
        }
    }
    
    //support for customized tone
    switch(inByte) {
      case '1': 
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        tone(SpeakerOutPin, 400+(inByte-48)*40);
        delay(80);
        noTone(SpeakerOutPin);
      break;
      case '0':
        tone(SpeakerOutPin, 400+(10)*40);
        delay(80);
        noTone(SpeakerOutPin);
      break;
    }
  }
}
void breathingLed(){
  
  float val = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
  if(getWindowState() == HIGH)
    analogWrite(LedOutPin, val);
  else
    analogWrite(LedOutPin, 0);
}
void simple_led_task(){
  smoothedDistance = lowpassFilter(outputValue[3], smoothedDistance, 0.25); 
  if(isHumanAround(smoothedDistance)){
    ledControl(HIGH);
    humanState = true;
  }
  else{
    if(humanState == true)
      //playDisappointedSound();
    ledControl(LOW);
    humanState = false;
  }
}

// Sensor tasks
// 
int getWindowState(){
  int reading = digitalRead(WindowInPin);
  if(reading == HIGH) return LOW;
  return HIGH;
}

// This is the calibration function for Sharp Distance Sensor 2Y0A02
// Note: this distance sensor can measure from 20 cm to 150 cm
int distanceCalibration(int input){
  // Valid raw data range is from 80 ~ 490
  if(input >= 80 && input <= 490)
    return 9462/(input - 16.92);
  return MAX_DISTANCE;
}
/* THRES40DB = 36.7;
   THRES45DB = 58.3;
   THRES50DB = 70.3;
   THRES55DB = 96.3;
   THRES60DB = 139.7;
   THRES65DB = 216;
   THRES70DB = 269;*/
int noiseLevel(int micVal){  
  calcBalanceMicVal(micVal);
  int difference = abs(balanceMicVal - micVal);
  if(difference< 3) 
      return 0;
  return 14.9620*(log(difference))-14.2486;
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
   temp += 4;
  if(Unit == Kelvin)
    return temp;
  else if(Unit == Celcius)
    return temp = temp - 273.15;            // Convert Kelvin to Celcius
  else if(Unit == Fahrenheit)
    return temp = (temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  
}

void ledControl(int input){
   if(input!= ledState){
      digitalWrite(LedOutPin, input);
      ledState = input;
   }
}

void flashLed(int pin, int times, int wait) {
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

void giveCandies(){
  digitalWrite(MotorOutPin, LOW);
  digitalWrite(MotorOutPin, HIGH);   // candy coming
  delay(100);
  digitalWrite(MotorOutPin, LOW);
  delay(100);
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
    int i;
    Serial.print(MachineID);
    Serial.print(",");
    for(i = 1; i < sensorNum -1; ++i){ Serial.print("0,");}
    Serial.println("0");
    delay(500);
  }
}

float lowpassFilter(float newValue, float oldValue, float alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}

boolean isHumanAround(int distance){
  return (distance < 500);
}
