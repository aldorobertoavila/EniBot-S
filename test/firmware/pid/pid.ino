#include <util/atomic.h>

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

// Control Gains
const float kp = 0.7988;
const float ki = 0.00001;
const float kd = 0.00095;

bool tcrt[3];
float distances[3];

int velocity;

float lowPassFilter;
float prevE;
float prevV;
float eIntegral;
long prevT;
long prevPos;

volatile long pos_volatile;

void readEncoderMotorA() {

}

void readEncoderMotorB() {
  int b = digitalRead(ENC_B2_PIN);
  int increment = 0;
  
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  
  pos_volatile = pos_volatile + increment;
}

void setup() {
  Serial.begin(9600);
  
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

void computeVelocity(int setPoint) {
  int pos;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {   
    pos = pos_volatile;
  }
  
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float deltaV = (pos - prevPos) / deltaT;
  float rawV = deltaV / 1000 * 60;

  prevT = currT;
  prevPos = pos;

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
  prevV = rawV;
  velocity = pwr;

  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(velocity);
}

void loop() {
  computeVelocity(100*(sin(micros()/1e6)>0));
  moveForward();
}
