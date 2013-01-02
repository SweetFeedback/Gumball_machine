  // Sensors' pin
const int windowNum = 6;
int windowPins[8] = {A0,A1,A2,A3,A4,A5,6,7};
int windowId[8] = {8,9,10,11,12,13};
int sensorValue[8] = {0};  // variable to store the value coming from the sensor
int outputValue[8] = {0};

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
    if(inByte == 'A'){
       printWindowId();
    }
    if (inByte == 'B') {
        Serial.print("data:");  
        for(i = 0; i < windowNum -1; i++){
            Serial.print(outputValue[i]);
              Serial.print(",");  
        }
        Serial.println(outputValue[i]);
    }
    
  }
}
void printWindowId(){
    int i;
    Serial.print("windowId:");  
    for(i = 0; i < windowNum-1; ++i){ 
      Serial.print(windowId[i]);  
      Serial.print(",");
    }
    Serial.println(windowId[i]);
}
void establishContact() {
  while (Serial.available() <= 0) {
    printWindowId();
    delay(500);    
  }

}


