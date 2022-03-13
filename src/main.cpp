#include <Arduino.h>

// TCRT5000 MODULES
#define TCRT_U1_PIN 2
#define TCRT_U2_PIN 3
#define TCRT_U3_PIN 4

// HC-SR04
#define ECHO_U1_PIN 5
#define ECHO_U2_PIN 6
#define ECHO_U3_PIN 7

#define TRIG_COM_PIN 8

// L298N
#define ENABLE_A_PIN 22
#define ENABLE_B_PIN 23
#define IN_A1_PIN 24
#define IN_A2_PIN 25
#define IN_B1_PIN 26
#define IN_B2_PIN 27

// Delay (us) for HC-SR04
const int TRIG_DELAY = 10;
unsigned int velocity = 95;

State previousCurrentState;
State currentState;

enum State {
  BRAKE,
  ERROR,
  FORWARD,
  REVERSE,
  STOP
};

void setState(State newState) {
  currentState = newState;
}

void readInput() {
  setState(FORWARD);
}

void onBrake() {
  brake();
}

void onError() {
  brake();
}

void onForward() {
  moveForward();
}

void onReverse() {
  moveBackward();
}

void onStop() {
  brake();
}

void setup() {
  pinMode(TCRT_U1_PIN, INPUT);
  pinMode(TCRT_U2_PIN, INPUT);
  pinMode(TCRT_U3_PIN, INPUT);
  
  pinMode(ECHO_U1_PIN, INPUT);
  pinMode(ECHO_U2_PIN, INPUT);
  pinMode(ECHO_U3_PIN, INPUT);

  pinMode(TRIG_COM_PIN, OUTPUT);
  
  pinMode(ENABLE_B_PIN, OUTPUT);
  pinMode(IN_A1_PIN, OUTPUT);
  pinMode(IN_A2_PIN, OUTPUT);
  pinMode(IN_B1_PIN, OUTPUT);
  pinMode(IN_B2_PIN, OUTPUT);
  pinMode(ENABLE_A_PIN, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  readInput();

   switch (currentState) {
    case BRAKE:
      onBrake();
      break;
    case ERROR:
      onError();
      break;
    case FORWARD:
      onForward();
      break;
    case REVERSE:
      onReverse();
      break;
    case STOP:
      onStop();
      break;
    default:
      break;
  }
}

float getUltrasonicDistance(int echoPin) {
  digitalWrite(TRIG_COM_PIN, HIGH);
  delayMicroseconds(TRIG_DELAY);
  digitalWrite(TRIG_COM_PIN, LOW);
  
  return pulseIn(echoPin, HIGH) / 58.2;
}

bool shouldTurnRight() {
  return digitalRead(TCRT_U1_PIN);
}

bool shouldTurnLeft() {
  return digitalRead(TCRT_U2_PIN);
}

bool shouldMoveForward() {
  return digitalRead(TCRT_U3_PIN);
}

void brake() {
  analogWrite(ENABLE_A_PIN, velocity);
  analogWrite(ENABLE_B_PIN, velocity);
  digitalWrite(IN_A1_PIN, LOW);
  digitalWrite(IN_A2_PIN, LOW);
  digitalWrite(IN_B1_PIN, LOW);
  digitalWrite(IN_B2_PIN, LOW);
}

void moveBackward() {
  analogWrite(ENABLE_A_PIN, velocity);
  analogWrite(ENABLE_B_PIN, velocity);
  digitalWrite(IN_A1_PIN, LOW);
  digitalWrite(IN_A2_PIN, HIGH);
  digitalWrite(IN_B1_PIN, LOW);
  digitalWrite(IN_B2_PIN, HIGH);
}

void moveForward() {
  analogWrite(ENABLE_A_PIN, velocity);
  analogWrite(ENABLE_B_PIN, velocity);
  digitalWrite(IN_A1_PIN, HIGH);
  digitalWrite(IN_A2_PIN, LOW);
  digitalWrite(IN_B1_PIN, HIGH);
  digitalWrite(IN_B2_PIN, LOW);
}

void moveLeft() {
  analogWrite(ENABLE_A_PIN, velocity);
  analogWrite(ENABLE_B_PIN, velocity);
  digitalWrite(IN_A1_PIN, HIGH);
  digitalWrite(IN_A2_PIN, LOW);
  digitalWrite(IN_B1_PIN, LOW);
  digitalWrite(IN_B2_PIN, HIGH);
}

void moveRight() {
  analogWrite(ENABLE_A_PIN, velocity);
  analogWrite(ENABLE_B_PIN, velocity);
  digitalWrite(IN_A1_PIN, LOW);
  digitalWrite(IN_A2_PIN, HIGH);
  digitalWrite(IN_B1_PIN, HIGH);
  digitalWrite(IN_B2_PIN, LOW);
}