#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "average.h"

#define LED_BUILTIN 2
#define GREEN_LED 21
#define FRONT_TRIGGER_PIN 2
#define FRONT_SENSOR_ECHO 4
#define BACK_TRIGGER_PIN 32
#define BACK_SENSOR_ECHO 34
#define DISABLE_WRITE 17
#define GROUND_TRUTH_A 16
#define GROUND_TRUTH_B 0
#define TIMEOUT 25000
//#define DEBUG

long duration;
int last_front_distance;
int last_back_distance;
int last_ground_truth1;
int last_ground_truth2;
bool sdcard_disabled;
MovingAverage front_ma = MovingAverage(10);
MovingAverage back_ma = MovingAverage(10);

/* Max' interrupt based stuff */
#define MEASURING 0
#define READY 1
#define SAMPLE_INTERVAL 25000 // in µs -> this is the hcsr04 timeout
#define TRIGGER_PIN FRONT_TRIGGER_PIN

// state variables that indicate whether a measurement is in progress or ready
volatile uint8_t state_front;
volatile uint8_t state_back;
// storing the flight time in us
volatile unsigned long flight_time_front;
volatile unsigned long flight_time_back;
// used to check if a new measuremnt should be executed
unsigned long check_time;

/* ISR to capture the flight time at the front sensor */
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

/* ISR to capture the flight time at the front sensor */
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
  pinMode(FRONT_TRIGGER_PIN, OUTPUT);
  pinMode(BACK_TRIGGER_PIN, OUTPUT);
  pinMode(DISABLE_WRITE, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_A, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_B, INPUT_PULLUP);

  // get inputs ready
  pinMode(FRONT_SENSOR_ECHO, INPUT_PULLDOWN);
  pinMode(BACK_SENSOR_ECHO, INPUT_PULLDOWN);
  // set interrupt modes, link to input pins and ISRs
  attachInterrupt(digitalPinToInterrupt(FRONT_SENSOR_ECHO), ISR_Front_Sensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_SENSOR_ECHO), ISR_Back_Sensor, CHANGE);

  // We could use higher baudrates!
  Serial.begin(9600);

  // setup SD-Card
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    sdcard_disabled = true;
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    sdcard_disabled = true;
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  sdcard_disabled = false;

  // trigger the sensors by generating a pwm signal 
  // both sensors have to be connected to the same trigger pin
  ledcSetup(0, 40, 8);            // channel 0, 40Hz, 8-bit resolution
  ledcAttachPin(TRIGGER_PIN, 0);  // attach TRIGGER_PIN to channel 0
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  if (sdcard_disabled) {
    return;
  }
  digitalWrite(GREEN_LED, HIGH);
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
  digitalWrite(GREEN_LED, LOW);
}

void appendMeasurement(float front_distance, float back_distance, int ground_truth1, int ground_truth2) {
  // To analyse the data more preciseley, we want to keep start and end time of all measurements.
  // This may be subject to change.
  char buf[128];
  sprintf(buf, "%.2f,%.2f,%d,%d\n", front_distance,back_distance, ground_truth1,ground_truth2);

  //if (last_front_distance != front_distance || last_back_distance != back_distance || last_ground_truth1 != ground_truth1 || last_ground_truth2 != ground_truth2) {
    appendFile(SD, "/log.csv", buf);
  //}
  // last_front_distance = front_distance;
  // last_back_distance = back_distance;
  // last_ground_truth1 = ground_truth1;
  // last_ground_truth2 = ground_truth2;
}

void loop() 
{
  if (digitalRead(DISABLE_WRITE)) {
    // digitalWrite(GREEN_LED, HIGH);
    Serial.println("Write protection, skipping everything");
    return;
  }
  digitalWrite(GREEN_LED, LOW);

  
  if (state_front == READY && state_back == READY) {
    // both sensors are ready, save the measurements
    float front_distance = flight_time_front / 1000.0 * 34.3 / 2.0;   // (Flugzeit [us] / 1000.0) [ms] * Schallgeschwindigkeit [cm/ms] / 2 = Distanz [cm]
    float back_distance = flight_time_back / 1000.0 * 34.3 / 2.0;
    appendMeasurement(front_distance, back_distance, digitalRead(GROUND_TRUTH_A), digitalRead(GROUND_TRUTH_B));
    #ifdef DEBUG
      // Prints the distance on the Serial Monitor
      Serial.print(millis());
      Serial.print(",");
      Serial.print(front_distance);
      Serial.print(",");
      Serial.print(back_distance);
      Serial.println();
    #endif
    state_back = !READY;
    state_front = !READY;
  }
}