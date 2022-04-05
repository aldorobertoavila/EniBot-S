// HC-SR04
#define ECHO_U1_PIN 23 // Left
#define ECHO_U2_PIN 25 // Front
#define ECHO_U3_PIN 27 // Right

#define TRIG_U1_PIN 22 // Left  
#define TRIG_U2_PIN 24 // Front
#define TRIG_U3_PIN 26 // Right

void setup() {
  Serial.begin(57600);
  
  pinMode(ECHO_U1_PIN, INPUT);
  pinMode(ECHO_U2_PIN, INPUT);
  pinMode(ECHO_U3_PIN, INPUT);

  pinMode(TRIG_U1_PIN, OUTPUT);
  pinMode(TRIG_U2_PIN, OUTPUT);
  pinMode(TRIG_U3_PIN, OUTPUT);
}

float getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) / 58.2;
}

void readSensors() {
  float u1 = getUltrasonicDistance(TRIG_U1_PIN, ECHO_U1_PIN);
  float u2 = getUltrasonicDistance(TRIG_U2_PIN, ECHO_U2_PIN);
  float u3 = getUltrasonicDistance(TRIG_U3_PIN, ECHO_U3_PIN);

  Serial.print(u1);
  Serial.print(", ");
  Serial.print(u2);
  Serial.print(", ");
  Serial.println(u3);

  delay(100);
}

void loop() {
  readSensors();
}
