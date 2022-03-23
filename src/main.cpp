#include <Arduino.h>
#include <util/atomic.h>

// TCRT5000
#define TCRT_U1_PIN 2 // Right
#define TCRT_U2_PIN 3 // Left
#define TCRT_U3_PIN 4 // Back

// HC-SR04
#define ECHO_U1_PIN 23 // Left
#define ECHO_U2_PIN 25 // Front
#define ECHO_U3_PIN 27 // Right

#define TRIG_U1_PIN 22 // Left
#define TRIG_U2_PIN 24 // Front
#define TRIG_U3_PIN 26 // Right

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

// Delay (us)
const unsigned int TRIG_CLEAR_DELAY = 2;
const unsigned int TRIG_HIGH_DELAY = 10;

// Velocity [0, 255] at 9V
const unsigned int FORWARD_VELOCITY = 95;
const unsigned int LEFTWARD_VELOCITY = 105;
const unsigned int RIGHTWARD_VELOCITY = 105;
const unsigned int REVERSE_VELOCITY = 95;

const unsigned int BACKWARD_TIMEOUT = 750;
// Delay (ms)
const unsigned int ULTRASONIC_DELAY = 20;

// Obstacle Distance (cm)
const float NEARBY_DISTANCE = 25.5;
const float SEARCH_DISTANCE = 40.25;

// Control Gains
const float kp = 0.9988;
const float ki = 0.00001;
const float kd = 0.00001;

unsigned long prevUltrasonicMillis;
unsigned long prevStateMillis;

int velocity;
float distance;

bool tcrt_u1;
bool tcrt_u2;
bool tcrt_u3;

State previousState;
State currentState;

float lowPassFilter;
float prevE;
float prevV;
float eIntegral;
long prevPos;
long prevT;

volatile long pos_volatile;
volatile long prevT_volatile;

void setState(State newState) {
  prevStateMillis = millis();
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
  setState(SEARCH);
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

void computeObstacle(int trigPin, int echoPin) {
  unsigned long currentMillis = millis();
  
  if(currentMillis - prevUltrasonicMillis > ULTRASONIC_DELAY) {
    distance = getUltrasonicDistance(TRIG_U2_PIN, ECHO_U2_PIN);    
    prevUltrasonicMillis = currentMillis;
  }

}

void computeVelocity(int setPoint) {
  int local_pos;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    local_pos = pos_volatile;
  }

  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float deltaV = (local_pos - prevPos) / deltaT;
  float rawV = deltaV / 1000 * 60;

  // Low-pass filter
  lowPassFilter = 0.89 * lowPassFilter + 0.009 * rawV + 0.010 * prevV;

  // Compute error
  float e = setPoint - lowPassFilter;
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

void onBrake() {
  computeVelocity(0);
  brake();
}

void onForward() {
  computeVelocity(FORWARD_VELOCITY);
  moveForward();

  tcrt_u1 = digitalRead(TCRT_U1_PIN);
  tcrt_u2 = digitalRead(TCRT_U2_PIN);

  if (tcrt_u1 || tcrt_u2) setState(REVERSE);
}

void onLeftward() {
  computeObstacle(TRIG_U2_PIN, ECHO_U2_PIN);
  computeVelocity(LEFTWARD_VELOCITY);
  moveLeft();

  if(distance <= NEARBY_DISTANCE) setState(FORWARD);
}

void onRightward() {
  computeObstacle(TRIG_U2_PIN, ECHO_U2_PIN);
  computeVelocity(RIGHTWARD_VELOCITY);
  moveRight();

  if(distance <= NEARBY_DISTANCE) setState(FORWARD);
}

void onReverse() {
  computeVelocity(REVERSE_VELOCITY);
  moveBackward();

  tcrt_u3 = digitalRead(TCRT_U3_PIN);

  if(tcrt_u3) {
    setState(FORWARD);
  }

  unsigned long currentMillis = millis();

  if(currentMillis - prevStateMillis > BACKWARD_TIMEOUT) {

    if(tcrt_u1 && tcrt_u2) {
      // if both, go turn around
      setState(SEARCH);
    } else if(tcrt_u1) {
      // if right, go right
      setState(LEFTWARD);
    } else if(tcrt_u2) {
      // if left, go right
      setState(RIGHTWARD);
    }

  }

}

void onSearch() {
  float distances[3] = {
    getUltrasonicDistance(TRIG_U1_PIN, ECHO_U1_PIN),
    getUltrasonicDistance(TRIG_U2_PIN, ECHO_U2_PIN),
    getUltrasonicDistance(TRIG_U3_PIN, ECHO_U3_PIN)
  };

  int index = -1;

  for (int i = 0; i < 3; i++) {
    float num = distances[i];
    if(index == -1 || (num < distances[index] && num <= SEARCH_DISTANCE)) index = i;
  }
 
  switch (index) {
  case 1:
    setState(FORWARD);
    break;
  case 2:
    setState(LEFTWARD);
    break;
  case 0:
  default:
    setState(RIGHTWARD);
    break;
  }
}

void onStop() {
  computeVelocity(0);
  brake();
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