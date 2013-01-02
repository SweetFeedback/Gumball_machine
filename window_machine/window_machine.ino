// Sensors' pin
const int windowNum = 2;
int windowPins[6] = {A0,A1,A2,A3,A4,A5};
int sensorValue[6] = {0};  // variable to store the value coming from the sensor
int outputValue[6] = {0};

//for group collaboration experiment
int count = 0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop() {
  getSensorData();
  serialCallResponse();
}

// Main Loop Tasks
void getSensorData() {
  // read the analog in value:
  for(int i = 0; i <= windowNum; i++){
    sensorValue[i] = analogRead(windowPins[i]);
    outputValue[i] = (sensorValue[i] > 512)?1:0;
  }

}

void serialCallResponse(){
  if(Serial.available() > 0) {
    int inByte = Serial.read();
    int i;
    if (inByte == 'B') {
        for(i = 0; i < windowNum -1; i++){
            Serial.print(outputValue[i]);
              Serial.print(",");  
        }
        Serial.println(outputValue[i]);
    }
    
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    int i;
    for(i = 1; i < windowNum; ++i){ Serial.print("-1,");}
    Serial.println("-1");
    delay(500);
  }
}


