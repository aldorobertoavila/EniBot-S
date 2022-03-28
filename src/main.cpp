#include <Arduino.h>
#include <SoftwareSerial.h>
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

// ESP-01
#define TX_PIN 14
#define RX_PIN 15

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

enum State {
  BRAKE,
  FORWARD,
  LEFTWARD,
  RIGHTWARD,
  REVERSE,
  SEARCH,
  STOP
};

SoftwareSerial esp8266(TX_PIN, RX_PIN);

// TCP/IP Communication
const int BAUDRATE = 115200;

// Delay (us)
const unsigned int TRIG_CLEAR_DELAY = 2;
const unsigned int TRIG_HIGH_DELAY = 10;

// Forward-Reverse
const unsigned int MAX_TRIES = 2;
const unsigned int BACKWARD_TIMEOUT = 600;

// Delay (ms)
const unsigned int ULTRASONIC_DELAY = 20;
const unsigned int COMMAND_DELAY = 100;

// Obstacle Distance (cm)
const float NEARBY_DISTANCE = 25.5;
const float SEARCH_DISTANCE = 40.25;

// Control Gains
const float kp = 0.9988;
const float ki = 0.00001;
const float kd = 0.00001;

// Velocity [0, 255] at 9V
unsigned int FORWARD_VELOCITY = 95;
unsigned int LEFTWARD_VELOCITY = 95;
unsigned int RIGHTWARD_VELOCITY = 95;
unsigned int REVERSE_VELOCITY = 95;

unsigned long prevUltrasonicMillis;
unsigned long previousSerialMillis;
unsigned long prevStateMillis;

int velocity;
int tries;

// Sensor Data
bool tcrt[3];
float distances[3];

Mode currentMode;
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

void setMode(Mode newMode) {
  currentMode = newMode;
}

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
  Serial.begin(BAUDRATE);
  esp8266.begin(BAUDRATE);

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

  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  
  // attachInterrupt(digitalPinToInterrupt(ENC_A1_PIN), readEncoderMotorA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2_PIN), readEncoderMotorB, RISING);

  setMode(AUTO);
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
    // Front
    distances[1] = getUltrasonicDistance(trigPin, echoPin);    
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

  int tcrt_u1 = tcrt[0];
  int tcrt_u2 = tcrt[1];

  if (tcrt_u1 || tcrt_u2) setState(REVERSE);
}

void onLeftward() {
  computeObstacle(TRIG_U2_PIN, ECHO_U2_PIN);
  computeVelocity(LEFTWARD_VELOCITY);
  moveLeft();

  // Front
  if(distances[1]  <= NEARBY_DISTANCE) setState(FORWARD);
}

void onRightward() {
  computeObstacle(TRIG_U2_PIN, ECHO_U2_PIN);
  computeVelocity(RIGHTWARD_VELOCITY);
  moveRight();

  // Front
  if(distances[1]  <= NEARBY_DISTANCE) setState(FORWARD);
}

void onReverse() {
  computeVelocity(REVERSE_VELOCITY);
  moveBackward();

  if(tries > MAX_TRIES) {
    tries = 0;

   if(tcrt[0]) {
      // if IR right, go right
      setState(LEFTWARD);
    } else if(tcrt[1]) {
      // if IR left, go right
      setState(RIGHTWARD);
    }
  
  }

  // Back
  if(tcrt[2]) {
    tries++;
    setState(FORWARD);
  }

  unsigned long currentMillis = millis();

  if(currentMillis - prevStateMillis > BACKWARD_TIMEOUT) {

    if(tcrt[0] && tcrt[1]) {
      // if both, go turn around
      setState(SEARCH);
    } else if(tcrt[0]) {
      // if right, go right
      setState(LEFTWARD);
    } else if(tcrt[1]) {
      // if left, go right
      setState(RIGHTWARD);
    }

  }

}

void onSearch() {
  distances[0] = getUltrasonicDistance(TRIG_U1_PIN, ECHO_U1_PIN); 
  distances[1] = getUltrasonicDistance(TRIG_U2_PIN, ECHO_U2_PIN);
  distances[2] = getUltrasonicDistance(TRIG_U3_PIN, ECHO_U3_PIN);

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

String* split(String str, char separator) {
  String* splits = {};
  int i = 0;

  while (str.length() > 0)
  {
    int index = str.indexOf(separator);
    
    if (index == -1) {
      splits[i++] = str;
      break;
    }
    else {
      splits[i++] = str.substring(0, index);
      str = str.substring(index + 1);
    }
  }
}

void readCommands() {
  unsigned long currentMillis = millis();

  if(currentMillis - previousSerialMillis > COMMAND_DELAY) {      
    
    if(esp8266.available() && esp8266.find('\n')) {
      String buf = esp8266.readStringUntil('\n');
      String* arr = split(buf, '=');
      
      int commandIndex = arr[0].toInt();
      int value = arr[1].toInt();

      Command command = static_cast<Command>(commandIndex);

      switch (command) {
        case GET_STATE:
          esp8266.println(currentState);
          break;
        case GET_TCRT:
          esp8266.println(tcrt[value]);
          break;
        case GET_U:
          esp8266.println(distances[value]);
          break;
        case GET_VELOCITY:
          esp8266.println(velocity);
          break;
        case SET_MODE:
          setMode(static_cast<Mode>(value));
          break;
        case SET_STATE:
          setState(static_cast<State>(value));
          break;
        case SET_VELOCITY_FORWARD:
          FORWARD_VELOCITY = value;
          break;
        case SET_VELOCITY_LEFTWARD:
          LEFTWARD_VELOCITY = value;
          break;
        case SET_VELOCITY_RIGHTWARD:
          RIGHTWARD_VELOCITY = value;
          break;
        case SET_VELOCITY_REVERSE:
          REVERSE_VELOCITY = value;
          break;
        default:
          break;
      }
    }
  }
}

void readInputs() {
  tcrt[0] = digitalRead(TCRT_U1_PIN);
  tcrt[1] = digitalRead(TCRT_U2_PIN);
  tcrt[2] = digitalRead(TCRT_U3_PIN);
}

void loop() {
  readInputs();
  readCommands();

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