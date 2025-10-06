#include <SoftwareSerial.h>
SoftwareSerial arduinoSerial(10,11);

const int ledPins[] = {
  53, 52, 51, 50, 49
}; // kitchen = 53, living = 52, bed = 51, dining = 50, cr = 49;
int led[5] = {0,0,0,0,0};
const int fanPins[] = {
  48, 47, 46
}; // living = 48, bed = 47, dining = 46;
int fan[3] = {0,0,0};
String EspFan[3];
const int irPins[] = {
  45, 44, 43, 42
}; // kitchen = 45, living = 44, bed = 43, dining = 42;
int ir[4];
String EspIR[4];
const int smokePins[] = {
  41, 40, 39
}; // kitchen = 41, living = 40, bed = 39; 
int smoke[3];
String EspSmoke[3];
const int Gkit = 38;
int Gas;
const int buzz = 37, valve = 36;
const int relay[] = {
  35, 34
}; // magnetic lock = 35, outlet = 34;
int rel[2];
const int ledpinCount = 5;
const int fanpinCount = 3;
const int irpinCount = 4;
const int smokepinCount = 3;
const int relaypinCount = 2;

String command, value;

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(9600);
  delay(2000);
  setPins();
}

void loop() {
  
  turnLedOn();

  turnFanOn();

  checkIR();

  checkSmoke();

  turnValveOn();

  turnRelayOn();

  sendesp32comm();

  receiveesp32comm();
}

void setPins(){
  for (int thisPin = 0; thisPin < ledpinCount; thisPin++){
    pinMode(ledPins[thisPin], OUTPUT);  
  }
  for (int thisPin = 0; thisPin < fanpinCount; thisPin++){
    pinMode(fanPins[thisPin], OUTPUT);  
  }
  for (int thisPin = 0; thisPin < irpinCount; thisPin++){
    pinMode(irPins[thisPin], INPUT);  
  }
  for (int thisPin = 0; thisPin < smokepinCount; thisPin++){
    pinMode(smokePins[thisPin], INPUT);  
  }
  pinMode(buzz, OUTPUT);
  pinMode(valve, OUTPUT);
  pinMode(Gkit, INPUT);
}

void turnLedOn() {
  for (int thisledPin = 0; thisledPin < ledpinCount; thisledPin++){
    if(led[thisledPin] == 1){
      digitalWrite(ledPins[thisledPin], HIGH);      
    } else {
      digitalWrite(ledPins[thisledPin], LOW);
    }
  }
}

void turnFanOn(){
  for (int thisfanPin = 0; thisfanPin < fanpinCount; thisfanPin++){
    if(fan[thisfanPin] == 1){
      digitalWrite(fanPins[thisfanPin], HIGH);      
    } else {
      digitalWrite(fanPins[thisfanPin], LOW);
    }
  }
}

void checkIR(){
  for (int thisirPin = 0; thisirPin < irpinCount; thisirPin++){
    if(digitalRead(irPins[thisirPin]) == LOW){
      ir[thisirPin] = 1;      
    } else {
      ir[thisirPin] = 0;
    }
  }
}

void checkSmoke(){
for (int thissmokePin = 0; thissmokePin < smokepinCount; thissmokePin++){
    if(digitalRead(smokePins[thissmokePin]) == LOW){
      smoke[thissmokePin] = 1;      
    } else {
      smoke[thissmokePin] = 0;
    }
  }
  if(digitalRead(Gkit) == LOW){
    Gas = 1;
  }
  else{
    Gas = 0;
  }
}

void turnValveOn(){
  if(smoke[0] == 1){ // kitchen & dining
    if(ir[0] == 1 || ir[1] == 1){ // kitchen  or dining
      digitalWrite(valve, HIGH);
      digitalWrite(buzz, HIGH);
      delay(5000);     
    } 
  }
  else if(smoke[1] == 1){ // dining & living 
    if(ir[1] == 1 || ir[2] == 1){ // dining or living
      digitalWrite(valve, HIGH);
      digitalWrite(buzz, HIGH);
      delay(5000);     
    }
  } 
  else if(smoke[2] == 1){ // bed
    if(ir[3] == 1){ // bed
      digitalWrite(valve, HIGH);
      digitalWrite(buzz, HIGH); 
      delay(5000);    
    }
  } 
    else{
      digitalWrite(valve, LOW);
      digitalWrite(buzz, LOW);
    }
}

void turnRelayOn(){
  for (int thisrelayPin = 0; thisrelayPin < relaypinCount; thisrelayPin++){
    if(rel[thisrelayPin] == 1){
      digitalWrite(relay[thisrelayPin], HIGH);
    } else{
      digitalWrite(relay[thisrelayPin], LOW);
    }   
  }
}

void sendesp32comm(){
  for (int thisirPin = 0; thisirPin < irpinCount; thisirPin++){
    EspIR[thisirPin] = String(ir[thisirPin]);
  }
  for (int thissmokePin = 0; thissmokePin < smokepinCount; thissmokePin++){
    EspSmoke[thissmokePin] = String(smoke[thissmokePin]);
  }

  String EspSendIR = EspIR[0] + EspIR[1] + EspIR[2] + EspIR[3]; 
  String EspSendSmoke = EspSmoke[0] + EspSmoke[1] + EspSmoke[2];

  Serial.println("Name1: " + command1 + " | Value1: " + value1);
  arduinoSerial.print("IR#" + EspSendIR + " Smoke#" + EspSendSmoke + " Gas#" + String(Gas));
  delay(100);
}

void receiveesp32comm(){

  if (arduinoSerial.available()) {
    String received = arduinoSerial.readStringUntil('\n');
    received.trim();  // Remove newline and extra spaces
    int separatorIndex = received.indexOf('#');

    if (separatorIndex != -1) {
        command = received.substring(0, separatorIndex);
        value = received.substring(separatorIndex + 1);

    } else {
        Serial.println("Invalid format (no '#')");
      }
  }
  if(command == "led"){
    char charArray[6]; // 5 digits + 1 for the null terminator
  
  // Copy the String into the character array
    value.toCharArray(charArray, 6);
    for(int thisledPin = 0; thisledPin < ledpinCount; thisledPin++){
      led[thisledPin] = charArray[thisledPin] - '0';
    } 
}
  else if (command == "fan") {
    char charArray[4];  // 3 fans + null
    value.toCharArray(charArray, 4);
    for (int i = 0; i < fanpinCount; i++) {
      fan[i] = charArray[i] - '0';
    }
  }
  else if (command == "relay") {
      int outlet = value.toInt();
      rel[1] = outlet;
    }
  command = "";
}