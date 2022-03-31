#include <Arduino.h>
#include <SimpleCLI.h>
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

enum Mode {
  AUTO,
  MANUAL
};

enum Sensor {
  ULTRASONIC,
  INFRARED
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

// Delay (us)
const unsigned int TRIG_CLEAR_DELAY = 2;
const unsigned int TRIG_HIGH_DELAY = 10;

// Forward-Reverse
const unsigned int MAX_TRIES = 2;
const unsigned int BACKWARD_TIMEOUT = 600;

// Delay (ms)
const unsigned int ULTRASONIC_DELAY = 20;

// Obstacle Distance (cm)
const float NEARBY_DISTANCE = 25.5;
const float SEARCH_DISTANCE = 40.25;

// Control Gains
const float KP = 0.9988;
const float KI = 0.00001;
const float KD = 0.00065;

String MODES[] = {
  "AUTO",
  "MANUAL"
};

String STATES[] = {
  "BRAKE",
  "FORWARD",
  "LEFTWARD",
  "RIGHTWARD",
  "REVERSE",
  "SEARCH",
  "STOP"
};

unsigned int MODES_SIZE = 2;
unsigned int STATES_SIZE = 7;

SimpleCLI cli;

Command MODE;
Command STATE;
Command SENSOR;
Command VELOCITY;

unsigned int forwardVelocity;
unsigned int leftwardVelocity;
unsigned int rightwardVelocity;
unsigned int reverseVelocity;

bool tcrt[3];
float distances[3];

Mode currentMode;
State currentState;

unsigned long prevUltrasonicMillis;
unsigned long previousSerialMillis;
unsigned long prevStateMillis;

int velocity;
int tries;

Mode previousMode;
State previousState;

float lowPassFilter;
float prevE;
float prevV;
float eIntegral;
long pos;
long prevT;
long prevPos;

volatile long pos_volatile;
volatile long prevT_volatile;

void setMode(Mode newMode) {
  previousMode = currentMode;
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

  if (b > 0)
    pos_volatile++;
  else
    pos_volatile--;
}

void errorCallback(cmd_error* e) {
  CommandError cmdError(e);

  Serial.print("ERROR: ");
  Serial.println(cmdError.toString());

  if (cmdError.hasCommand()) {
    Serial.print("Did you mean \"");
    Serial.print(cmdError.getCommand().toString());
    Serial.println("\"?");
  }
}

Mode getModeByName(String modeName) {
  if (modeName.equals("AUTO")) return AUTO;
  else if (modeName.equals("MANUAL")) return MANUAL;
  return -1;
}

Sensor getSensorByName(String sensorName) {
  if (sensorName.equals("ULTRASONIC")) return ULTRASONIC;
  else if (sensorName.equals("INFRARED")) return INFRARED;
  return -1;
}

State getStateByName(String stateName) {
  for (int i = 0; i < STATES_SIZE; i++) {
    if (stateName.equals(STATES[i])) return i;
  }
  return -1;
}

void setup() {
  Serial.begin(9600);

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

  cli.setOnError(errorCallback);

  MODE = cli.addCommand("mode");
  MODE.addArgument("set", "AUTO");
  MODE.addFlagArgument("get");

  SENSOR = cli.addCommand("sensor");
  SENSOR.addArgument("type");
  SENSOR.addArgument("i");
  SENSOR.addArgument("set", 0);
  SENSOR.addFlagArgument("get");

  STATE = cli.addCommand("state");
  STATE.addArgument("set", "BRAKE");
  STATE.addFlagArgument("get");

  VELOCITY = cli.addCommand("velocity");
  VELOCITY.addArgument("direction");
  VELOCITY.addArgument("setpoint", 0);
  VELOCITY.addFlagArgument("get");

  forwardVelocity = 100;
  leftwardVelocity = 100;
  rightwardVelocity = 100;
  reverseVelocity = 100;

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

void computeVelocity(int setPoint) {
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float deltaV = (pos - prevPos) / deltaT;
  float rawV = deltaV / 1000 * 60;

  // Low-pass filter
  lowPassFilter = 0.89 * lowPassFilter + 0.009 * rawV + 0.010 * prevV;

  // Compute error
  float e = setPoint - lowPassFilter;
  float dError = (e  - prevE) / deltaT;

  eIntegral = eIntegral + (e * deltaT);

  // Compute u
  float u = KP * e + KI * eIntegral + KD * dError;
  int pwr = (int) fabs(u);

  if (pwr > 255) {
    pwr = 255;
  } else if (pwr < 0) {
    pwr = 0;
  }

  prevE = e;
  prevT = currT;
  prevV = rawV;
  prevPos = pos;
  velocity = pwr;
}

void onBrake() {
  computeVelocity(0);
  brake();
}

void onForward() {
  computeVelocity(forwardVelocity);
  moveForward();

  if (currentMode != MANUAL) {
    int tcrt_u1 = tcrt[0];
    int tcrt_u2 = tcrt[1];

    if (tcrt_u1 || tcrt_u2) setState(REVERSE);
  }
}

void onLeftward() {
  computeVelocity(leftwardVelocity);
  moveLeft();

  // Front
  if (currentMode != MANUAL && distances[1] <= NEARBY_DISTANCE) setState(FORWARD);
}

void onRightward() {
  computeVelocity(rightwardVelocity);
  moveRight();

  // Front
  if (currentMode != MANUAL && distances[1] <= NEARBY_DISTANCE) setState(FORWARD);
}

void onReverse() {
  computeVelocity(reverseVelocity);
  moveBackward();

  if (currentMode != MANUAL) {
    if (tries > MAX_TRIES) {
      tries = 0;

      if (tcrt[0]) {
        // if IR right, go right
        setState(LEFTWARD);
      } else if (tcrt[1]) {
        // if IR left, go right
        setState(RIGHTWARD);
      }
    }

    // Back
    if (tcrt[2]) {
      tries++;
      setState(FORWARD);
    }

    unsigned long currentMillis = millis();

    if (currentMillis - prevStateMillis > BACKWARD_TIMEOUT) {

      if (tcrt[0] && tcrt[1]) {
        // if both, go turn around
        setState(SEARCH);
      } else if (tcrt[0]) {
        // if right, go right
        setState(LEFTWARD);
      } else if (tcrt[1]) {
        // if left, go right
        setState(RIGHTWARD);
      }

    }
  }
}

void onSearch() {
  computeVelocity(0);
  brake();

  if (currentMode != MANUAL) {
    int index = -1;

    for (int i = 0; i < 3; i++) {
      float num = distances[i];
      if (index == -1 || (num < distances[index] && num <= SEARCH_DISTANCE)) index = i;
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
}

void onStop() {
  computeVelocity(0);
  brake();
}

void modeCommand(Command cmd) {
  Argument setterArg = cmd.getArgument("set");
  String setterValue = setterArg.getValue();
  bool isSetterSet = setterArg.isSet();
  Mode newMode = getModeByName(setterValue);

  Argument getterArg = cmd.getArgument("get");
  bool isGetterSet = getterArg.isSet();

  if (newMode != -1 || isGetterSet) {
    if (isSetterSet) setMode(newMode);
    if (isGetterSet) Serial.println(MODES[isSetterSet ? newMode : currentMode]);
  } else {
    Serial.print("ERROR: Invalid Mode <");
    Serial.print(setterValue);
    Serial.print(">, must be any of [ ");
    for (int i = 0; i < MODES_SIZE; i++) {
      Serial.print(MODES[i]);
      if (i < MODES_SIZE - 1) Serial.print(", ");
    }
    Serial.println("].");
  }
}

void sensorCommand(Command cmd) {
  Argument sensorArg = cmd.getArgument("type");
  String sensorValue = sensorArg.getValue();
  Sensor type = getSensorByName(sensorValue);

  Argument indexArg = cmd.getArgument("i");
  int i = indexArg.getValue().toInt();

  Argument setterArg = cmd.getArgument("set");
  float newValue = setterArg.getValue().toFloat();
  int newInt = setterArg.getValue().toInt();
  bool isSetterSet = setterArg.isSet();

  Argument getterArg = cmd.getArgument("get");
  bool isGetterSet = getterArg.isSet();

  switch (type) {
    case ULTRASONIC:
      if (i >= 0 && i <= 2) {
        if (isSetterSet) distances[i] = newValue;
        if (isGetterSet) Serial.println(distances[i]);
      } else {
        Serial.print("ERROR: Invalid Ultrasonic Index <");
        Serial.print(i);
        Serial.println(">, must be between [0, 2].");
      }
      break;
    case INFRARED:
      if (i >= 0 && i <= 2) {
        if (isSetterSet) {
          if (newInt == 0 || newInt == 1) {
            tcrt[i] = newInt;
          } else {
            Serial.print("ERROR: Invalid Value for Infrared <");
            Serial.print(newInt);
            Serial.println(">, must be [0, 1].");
            return;
          }
        }
        if (isGetterSet) Serial.println(tcrt[i]);
      } else {
        Serial.print("ERROR: Invalid infrared index <");
        Serial.print(i);
        Serial.println(">, must be between [0, 2].");
      }
      break;
    default:
      Serial.print("ERROR: Invalid sensor <");
      Serial.print(sensorValue);
      Serial.println(">, must be either \"ULTRASONIC\" or \"INFRARED\".");
      break;
  }
}

void stateCommand(Command cmd) {
  Argument setterArg = cmd.getArgument("set");
  String setterValue = setterArg.getValue();
  bool isSetterSet = setterArg.isSet();
  State newState = getStateByName(setterValue);

  Argument getterArg = cmd.getArgument("get");
  bool isGetterSet = getterArg.isSet();

  if (newState != -1 || isGetterSet) {
    if (isSetterSet) setState(newState);
    if (isGetterSet) Serial.println(STATES[isSetterSet ? newState : currentState]);
  } else {
    Serial.print("ERROR: Invalid State <");
    Serial.print(setterValue);
    Serial.print(">, must be any of [ ");
    for (int i = 0; i < STATES_SIZE; i++) {
      Serial.print(STATES[i]);
      if (i < STATES_SIZE - 1) Serial.print(", ");
    }
    Serial.println("].");
  }
}

void velocityCommand(Command cmd) {
  Argument dirArg = cmd.getArgument("direction");
  String str = dirArg.getValue();
  char dirValue = str[0];

  Argument setterArg = cmd.getArgument("setpoint");
  int newValue = setterArg.getValue().toInt();
  bool isSetterSet = setterArg.isSet();

  Argument getterArg = cmd.getArgument("get");
  bool isGetterSet = getterArg.isSet();

  bool isValid = str.length() == 1;
  int* addr;

  switch (dirValue) {
    case 'f':
      addr = &forwardVelocity;
      break;
    case 'r':
      addr = &rightwardVelocity;
      break;
    case 'l':
      addr = &leftwardVelocity;
      break;
    case 'b':
      addr = &reverseVelocity;
      break;
    default:
      isValid = isValid && false;
      break;
  }

  if (isValid) {
    if (isSetterSet) {
      if (newValue >= 0 && newValue <= 255) *addr = newValue;
      else {
        Serial.print("ERROR: Invalid Set Point for Velocity <");
        Serial.print(newValue);
        Serial.println(">, must be between [0, 225].");
        return;
      }
    }
    if (isGetterSet) Serial.println(*addr);
  } else {
    Serial.print("ERROR: Invalid Direction <");
    Serial.print(dirValue);
    Serial.println(">, must be any of ['f', 'r', 'l', 'b'].");
  }
}

void readCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.print("# ");
    Serial.println(input);
    cli.parse(input);

    Command cmd = cli.getCmd();

    if (cmd == MODE) modeCommand(cmd);
    else if (cmd == SENSOR) sensorCommand(cmd);
    else if (cmd == STATE) stateCommand(cmd);
    else if (cmd == VELOCITY) velocityCommand(cmd);
  }
}

void readSensors() {
  unsigned long currentMillis = millis();

  if (currentMillis - prevUltrasonicMillis > ULTRASONIC_DELAY) {
    distances[0] = getUltrasonicDistance(TRIG_U1_PIN, ECHO_U1_PIN);
    distances[1] = getUltrasonicDistance(TRIG_U2_PIN, ECHO_U2_PIN);
    distances[2] = getUltrasonicDistance(TRIG_U3_PIN, ECHO_U3_PIN);
    prevUltrasonicMillis = currentMillis;
  }

  tcrt[0] = !digitalRead(TCRT_U1_PIN);
  tcrt[1] = !digitalRead(TCRT_U2_PIN);
  tcrt[2] = !digitalRead(TCRT_U3_PIN);
}

void updateState() {

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

void loop() {
  // update before CLI
  readSensors();
  // update command-line interface
  readCommands();

  // update FMS
  updateState();
}