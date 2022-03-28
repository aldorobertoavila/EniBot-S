#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ModbusTCPSlave.h>
#include <Ticker.h>

const boolean DEBUG = false;
const int TIMEOUT = 100;

char *network = "";
char *password = "";

byte ip[] = { 0, 0, 0, 0 };
byte gateway[] = { 0, 0, 0, 0 };
byte subnet[] = { 0, 0, 0, 0 };

ModbusTCPSlave tcp;
Ticker ticker;

int control;
int state;
int tcrt_u1;
int tcrt_u2;
int tcrt_u3;
int u1;
int u2;
int u3;

int velocity_forward;
int velocity_leftward;
int velocity_rightward;
int velocity_reverse;

enum Command {
  GET_STATE,
  GET_TCRT,
  GET_U,
  GET_VELOCITY,
  SET_MODE,
  SET_STATE,
  SET_VELOCITY_FORWARD,
  SET_VELOCITY_LEFTWARD,
  SET_VELOCITY_RIGHTWARD,
  SET_VELOCITY_REVERSE
};

enum Mode {
  AUTO,
  MANUAL
};

enum TCRT5000 {
  TCRT_U1,
  TCRT_U2,
  TCRT_U3
};

enum HCSR04 {
  HCSR_U1,
  HCSR_U2,
  HCSR_U3
};

enum Registry {
  CONTROL,
  STATE,
  INFRARED1,
  INFRARED2,
  INFRARED3,
  ULTRASONIC1,
  ULTRASONIC2,
  ULTRASONIC3,
  VELOCITY_FORWARD,
  VELOCITY_LEFTWARD,
  VELOCITY_RIGHTWARD,
  VELOCITY_REVERSE
};

void readRegister(Registry regis, int* value) {
  *value = tcp.MBHoldingRegister[regis];
}

void writeRegister(Registry regis, int value) {
  tcp.MBHoldingRegister[regis] = value;
}

float getFloat(int command, int argument) {
  return sendCommand(command, argument).toFloat();
}

int getInteger(int command, int argument) {
  return sendCommand(command, argument).toInt();
}

String sendCommand(int command, int argument) {
  long currentMillis = millis();                                      
  String response = "";                                             

  Serial.println(command);

  while(currentMillis + TIMEOUT > millis()) {      
    while(Serial.available()) {                                   
      response+=Serial.read();                                                  
    }
  }
    
  if(DEBUG) Serial.println(response); 
  return response;                                                  
}

void bus() {
  readRegister(CONTROL, &control);

  if(control > 0) {
    sendCommand(SET_MODE, MANUAL);
    readRegister(STATE, &state);
    readRegister(VELOCITY_FORWARD, &velocity_forward);
    readRegister(VELOCITY_LEFTWARD, &velocity_leftward);
    readRegister(VELOCITY_RIGHTWARD, &velocity_rightward);
    readRegister(VELOCITY_REVERSE, &velocity_reverse);
  } else {
    sendCommand(SET_MODE, AUTO);
    writeRegister(STATE, state);
    writeRegister(VELOCITY_FORWARD, velocity_forward);
    writeRegister(VELOCITY_LEFTWARD, velocity_leftward);
    writeRegister(VELOCITY_RIGHTWARD, velocity_rightward);
    writeRegister(VELOCITY_REVERSE, velocity_reverse);
  }

  tcrt_u1 = getInteger(GET_TCRT, TCRT_U1);
  tcrt_u2 = getInteger(GET_TCRT, TCRT_U2);
  tcrt_u3 = getInteger(GET_TCRT, TCRT_U3);
  u1 = getFloat(GET_U, HCSR_U1);
  u2 = getFloat(GET_U, HCSR_U2);
  u3 = getFloat(GET_U, HCSR_U3);
  
  writeRegister(INFRARED1, tcrt_u1);
  writeRegister(INFRARED2, tcrt_u2);
  writeRegister(INFRARED3, tcrt_u3);
  writeRegister(ULTRASONIC1, u1);
  writeRegister(ULTRASONIC2, u2);
  writeRegister(ULTRASONIC3, u3);
}

void setup() {
  Serial.begin(115200);
  tcp.begin(network, password, ip, gateway, subnet);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  ticker.attach_ms(40, bus);
}

void loop() {
  tcp.Run();
  delay(10);
}