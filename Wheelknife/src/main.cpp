#include <Arduino.h>

#define LED_BUILTIN 2
#define GREEN_LED 19
#define FRONT_SENSOR_TRIGGER 2
#define FRONT_SENSOR_ECHO 4
#define BACK_SENSOR_TRIGGER 32
#define BACK_SENSOR_ECHO 34

long duration;
int front_distance;
int back_distance;

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(FRONT_SENSOR_TRIGGER, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(BACK_SENSOR_TRIGGER, OUTPUT);
  pinMode(BACK_SENSOR_ECHO, INPUT);
  Serial.begin(9600);
}

int get_distance(int triggerPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  return duration*0.034/2;
}

void loop() {
  front_distance = get_distance(FRONT_SENSOR_TRIGGER, FRONT_SENSOR_ECHO);
  back_distance = get_distance(BACK_SENSOR_TRIGGER, BACK_SENSOR_ECHO);
  // Prints the distance on the Serial Monitor
  Serial.print(front_distance);
  Serial.print(",");
  Serial.print(back_distance);
  Serial.println();
}