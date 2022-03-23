#include <Arduino.h>
#include <util/atomic.h>

// TCRT5000
#define TCRT_U1_PIN 2 // Right
#define TCRT_U2_PIN 3 // Left
#define TCRT_U3_PIN 4 // Back

// HC-SR04
#define ECHO_U1_PIN 23
#define ECHO_U2_PIN 25
#define ECHO_U3_PIN 27

#define TRIG_U1_PIN 22
#define TRIG_U2_PIN 24
#define TRIG_U3_PIN 26

// L298N
#define ENABLE_A_PIN 8
#define IN1_PIN 9
#define IN2_PIN 10
#define IN3_PIN 11
#define IN4_PIN 12
#define ENABLE_B_PIN 13

// Encoders
#define ENC_A1_PIN 18
#define ENC_B1_PIN 19
#define ENC_A2_PIN 20
#define ENC_B2_PIN 21

enum State {
  BRAKE,
  FORWARD,
  LEFTWARD,
  RIGHTWARD,
  REVERSE,
  SEARCH,
  STOP
};

// Delay (us) for HC-SR04
const int TRIG_CLEAR_DELAY = 2;
const int TRIG_HIGH_DELAY = 10;

// Velocity [0, 255] at 9V
const int FORWARD_VELOCITY = 100;
const int LEFTWARD_VELOCITY = 100;
const int RIGHTWARD_VELOCITY = 100;
const int REVERSE_VELOCITY = 100;

// Control Gains
const float kp = 0.9988;
const float ki = 0.00001;
const float kd = 0.00001;

unsigned long previousStateMillis;
unsigned int velocity;

bool tcrt_u1;
bool tcrt_u2;
bool tcrt_u3;

State previousState;
State currentState;

float prevE;
float prevV;
float eIntegral;
long prevPos;
long prevT;

volatile long pos_volatile;
volatile long prevT_volatile;
volatile float velocity_volatile;

void setState(State newState) {
  previousStateMillis = millis();
  previousState = currentState;
  currentState = newState;
}

void readEncoderMotorA() {

}

void readEncoderMotorB() {
  int b = digitalRead(ENC_B2_PIN);

  if(b > 0)
    pos_volatile++;
  else
    pos_volatile--;
}

void updateState() {

  if (tcrt_u1 && tcrt_u2) {
    setState(REVERSE);
  }

}

void setup() {
  pinMode(TCRT_U1_PIN, INPUT);
  pinMode(TCRT_U2_PIN, INPUT);
  pinMode(TCRT_U3_PIN, INPUT);

  pinMode(ECHO_U1_PIN, INPUT);
  pinMode(ECHO_U2_PIN, INPUT);
  pinMode(ECHO_U3_PIN, INPUT);

  pinMode(TRIG_U1_PIN, OUTPUT);
  pinMode(TRIG_U2_PIN, OUTPUT);
  pinMode(TRIG_U3_PIN, OUTPUT);

  pinMode(ENABLE_A_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENABLE_B_PIN, OUTPUT);

  pinMode(ENC_A1_PIN, INPUT_PULLUP);
  pinMode(ENC_B1_PIN, INPUT_PULLUP);
  pinMode(ENC_A2_PIN, INPUT_PULLUP);
  pinMode(ENC_B2_PIN, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(ENC_A1_PIN), readEncoderMotorA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2_PIN), readEncoderMotorB, RISING);

  Serial.begin(9600);
}

float getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(TRIG_CLEAR_DELAY);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(TRIG_HIGH_DELAY);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) / 58.2;
}

void brake() {
  analogWrite(ENABLE_A_PIN, velocity);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENABLE_B_PIN, velocity);
}

void moveBackward() {
  analogWrite(ENABLE_A_PIN, velocity);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENABLE_B_PIN, velocity);
}

void moveForward() {
  analogWrite(ENABLE_A_PIN, velocity);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENABLE_B_PIN, velocity);
}

void moveLeft() {
  analogWrite(ENABLE_A_PIN, velocity);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENABLE_B_PIN, velocity);
}

void moveRight() {
  analogWrite(ENABLE_A_PIN, velocity);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENABLE_B_PIN, velocity);
}

void onBrake() {
  computeVelocity(0);
  brake();
}

void onForward() {
  computeVelocity(FORWARD_VELOCITY);
  moveForward();
}

void onLeftward() {
  computeVelocity(LEFTWARD_VELOCITY);
  moveLeft();
}

void onRightward() {
  computeVelocity(RIGHTWARD_VELOCITY);
  moveRight();
}

void onReverse() {
  computeVelocity(REVERSE_VELOCITY);
  moveBackward();
}

void onSearch() {
  
}

void onStop() {
  computeVelocity(0);
  brake();
}

void computeVelocity(int setPoint) {
  int local_pos;
  float local_velocity;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    local_pos = pos_volatile;
    local_velocity = velocity_volatile;
  }

  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float deltaV = (local_pos - prevPos) / deltaT;
  float rawV = deltaV / 1000 * 60;

  // Low-pass filter
  float filter = 0.89 * filter + 0.009 * rawV + 0.010 * prevV;

  // Compute e
  float e = setPoint - filter;
  float dError = (e  - prevE) / deltaT;

  eIntegral = eIntegral + (e * deltaT);

  // Compute u
  float u = kp * e + ki * eIntegral + kd * dError;
  int pwr = (int) fabs(u);

  if (pwr > 255) {
    pwr = 255;
  } else if(pwr < 0) {
    pwr = 0;
  }

  prevE = e;
  prevT = currT;
  prevV = rawV;
  prevPos = local_pos;
  velocity = pwr;

  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(velocity);
}

void loop() {
  switch (currentState) {
    case BRAKE:
      onBrake();
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
    case SEARCH:
      onSearch();
      break;
    case STOP:
      onStop();
      break;
    default:
      break;
  }
}