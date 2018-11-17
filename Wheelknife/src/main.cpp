#include <Arduino.h>

#define LED_BUILTIN 2
#define GREEN_LED 19

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

void loop() {
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(GREEN_LED, LOW);

  // wait for a second
  delay(1000);

  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(GREEN_LED, HIGH);

   // wait for a second
  delay(1000);
}