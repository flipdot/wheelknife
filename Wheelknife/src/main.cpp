#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "average.h"

#define LED_BUILTIN 2
#define GREEN_LED 21
#define FRONT_SENSOR_TRIGGER 2
#define FRONT_SENSOR_ECHO 4
#define BACK_SENSOR_TRIGGER 32
#define BACK_SENSOR_ECHO 34
#define DISABLE_WRITE 17
#define GROUND_TRUTH_A 16
#define GROUND_TRUTH_B 0
#define TIMEOUT 25000

long duration;
int last_front_distance;
int last_back_distance;
int last_ground_truth1;
int last_ground_truth2;
MovingAverage front_ma = MovingAverage(10);
MovingAverage back_ma = MovingAverage(10);

// max' interrupt based stuff
#define MEASURING 0
#define READY 1
#define SAMPLE_INTERVAL 25500 // in µs -> this is the hcsr04 timeout
#define TRIGGER_PIN FRONT_SENSOR_TRIGGER

volatile uint8_t state_front;
volatile uint8_t state_back;
volatile unsigned long flight_time_front;
volatile unsigned long flight_time_back;
unsigned long check_time;
uint16_t front_distance;
uint16_t back_distance;

void ISR_Front_Sensor()
{
  if (digitalRead(FRONT_SENSOR_ECHO)) {
    flight_time_front = micros();
    state_front = MEASURING;
  }
  else {
    flight_time_front = micros() - flight_time_front;
    state_front = READY;
  }
}

void ISR_Back_Sensor()
{
  if (digitalRead(BACK_SENSOR_ECHO)) {
    flight_time_back = micros();
    state_back = MEASURING;
  }
  else {
    flight_time_back = micros() - flight_time_front;
    state_back = READY;
  }
}

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(FRONT_SENSOR_TRIGGER, OUTPUT);
  // pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(BACK_SENSOR_TRIGGER, OUTPUT);
  // pinMode(BACK_SENSOR_ECHO, INPUT);
  pinMode(DISABLE_WRITE, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_A, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_B, INPUT_PULLUP);

  // get inputs ready for interrupts
  pinMode(FRONT_SENSOR_ECHO, INPUT_PULLDOWN);
  pinMode(BACK_SENSOR_ECHO, INPUT_PULLDOWN);
  // set interrupt modes, link to input pins and ISRs
  attachInterrupt(digitalPinToInterrupt(FRONT_SENSOR_ECHO), ISR_Front_Sensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_SENSOR_ECHO), ISR_Back_Sensor, CHANGE);

  // setup Serial
  Serial.begin(9600);

  // setup SD-Card
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // init check_time
  check_time = micros();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}

void appendMeasurement(int start_time, int mid_time, int end_time, int front_distance, int back_distance, int ground_truth1, int ground_truth2) {
  // To analyse the data more preciseley, we want to keep start and end time of all measurements.
  // This may be subject to change.
  char buf[128];
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d\n", start_time,mid_time,end_time, front_distance,back_distance, ground_truth1,ground_truth2);

  //if (last_front_distance != front_distance || last_back_distance != back_distance || last_ground_truth1 != ground_truth1 || last_ground_truth2 != ground_truth2) {
    appendFile(SD, "/log.csv", buf);
  //}
  last_front_distance = front_distance;
  last_back_distance = back_distance;
  last_ground_truth1 = ground_truth1;
  last_ground_truth2 = ground_truth2;
}

int get_distance(int triggerPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, TIMEOUT);
  // Calculating the distance
  return duration*0.034/2;
}

void trigger_hcsr04() 
{
  // Clears the trigPin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
}

void loop() {
  unsigned long startTime;
  
  if (micros() - check_time >= SAMPLE_INTERVAL) {
    // send burst 
    trigger_hcsr04();
    startTime = micros();
    check_time = micros();
  }

  int disable_write = digitalRead(DISABLE_WRITE);
  if (disable_write) {
    digitalWrite(GREEN_LED, HIGH);
    return;
  }
  digitalWrite(GREEN_LED, LOW);

  if (state_front == READY && state_back == READY) {
    // both sensors are ready, save the measurements
    int front_distance = flight_time_front * 0.034 / 2;
    int back_distance = flight_time_back * 0.034 / 2;
    appendMeasurement(startTime, /*midTime*/ startTime - flight_time_front, /*endTime*/ startTime - flight_time_back, front_distance, back_distance, digitalRead(GROUND_TRUTH_A), digitalRead(GROUND_TRUTH_B));
  }

  #ifdef DEBUG
    // Prints the distance on the Serial Monitor
    Serial.print(millis());
    Serial.print(",");
    Serial.print(front_distance);
    Serial.print(",");
    Serial.print(back_distance);
    Serial.println();
  #endif
}