#include <Arduino.h>

// TCRT5000
#define TCRT_U1_PIN 2 // Right
#define TCRT_U2_PIN 3 // Left
#define TCRT_U3_PIN 4 // Back

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

// Encoders
#define ENC_A1_PIN 28
#define ENC_B1_PIN 29
#define ENC_A2_PIN 30
#define ENC_B2_PIN 31

// Delay (us) for HC-SR04
const int TRIG_CLEAR_DELAY = 2;
const int TRIG_HIGH_DELAY = 10;

unsigned long previousStateMillis;
unsigned int velocity;

bool tcrt_u1;
bool tcrt_u2;
bool tcrt_u3;

State previousState;
State currentState;

enum State {
  BRAKE,
  ERROR,
  FORWARD,
  LEFTWARD,
  RIGHTWARD,
  REVERSE,
  STOP
};

void setState(State newState) {
  previousStateMillis = millis();
  previousState = currentState;
  currentState = newState;
}

void readInput() {
  tcrt_u1 = digitalRead(TCRT_U1_PIN);
  tcrt_u2 = digitalRead(TCRT_U2_PIN);
  tcrt_u3 = digitalRead(TCRT_U3_PIN);

  // getUltrasonicDistance(ECHO_U1_PIN);
  // getUltrasonicDistance(ECHO_U2_PIN);
  // getUltrasonicDistance(ECHO_U3_PIN);
}

void updateState() {
  
  if(tcrt_u1 && tcrt_u2) {
    setState(REVERSE);
  }

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

void onLeftward() {
  moveLeft();
}

void onRightward() {
  moveRight();
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

  pinMode(ENABLE_A_PIN, OUTPUT);
  pinMode(ENABLE_B_PIN, OUTPUT);
  pinMode(IN_A1_PIN, OUTPUT);
  pinMode(IN_A2_PIN, OUTPUT);
  pinMode(IN_B1_PIN, OUTPUT);
  pinMode(IN_B2_PIN, OUTPUT);

  pinMode(ENC_A1_PIN, INPUT);
  pinMode(ENC_B1_PIN, INPUT);
  pinMode(ENC_A2_PIN, INPUT);
  pinMode(ENC_B2_PIN, INPUT);

  Serial.begin(9600);

  velocity = 100;
}

void loop() {
  readInput();
  updateState();

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
    case LEFTWARD:
      onLeftward();
      break;
    case RIGHTWARD:
      onRightward();
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
  digitalWrite(TRIG_COM_PIN, LOW);
  delayMicroseconds(TRIG_CLEAR_DELAY);
  digitalWrite(TRIG_COM_PIN, HIGH);
  delayMicroseconds(TRIG_HIGH_DELAY);
  digitalWrite(TRIG_COM_PIN, LOW);
  
  return pulseIn(echoPin, HIGH) / 58.2;
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