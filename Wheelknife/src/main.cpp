#include <Arduino.h>

#define LED_BUILTIN 2
#define GREEN_LED 19
#define FRONT_SENSOR_TRIGGER 2
#define FRONT_SENSOR_ECHO 4
#define BACK_SENSOR_TRIGGER 32
#define BACK_SENSOR_ECHO 34

long duration;
int distance;

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

void loop() {
  // Clears the trigPin
  digitalWrite(FRONT_SENSOR_TRIGGER, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(FRONT_SENSOR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_SENSOR_TRIGGER, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(FRONT_SENSOR_ECHO, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}