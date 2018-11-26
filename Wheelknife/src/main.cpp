#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "average.h"

#define LED_BUILTIN       2
#define GREEN_LED         21
// #define TRIGGER_PIN_FRONT 2
// #define TRIGGER_PIN_BACK  32
#define TRIGGER_PIN       2       // both sensors should be connected to a singel trigger signal now!
#define ECHO_PIN_FRONT    4
#define ECHO_PIN_BACK     34
#define DISABLE_WRITE     17
#define GROUND_TRUTH_A    16
#define GROUND_TRUTH_B    0
//#define DEBUG

bool sdcard_disabled;
MovingAverage front_ma = MovingAverage(10);
MovingAverage back_ma = MovingAverage(10);

/* Max' interrupt based stuff */
#define MEASURING 0
#define READY 1
#define SAMPLE_INTERVAL 25e-3 // in s -> this is the hcsr04 timeout
#define TRIGGER_PWM_FREQUENCY 1.0 / SAMPLE_INTERVAL
#define CHANNEL 0
#define DUTYCYCLE 127

// state variables that indicate whether a measurement is in progress or ready
volatile uint8_t state_front;
volatile uint8_t state_back;
// storing the flight time in us
volatile unsigned long flight_time_front;
volatile unsigned long flight_time_back;
// measuremant start times
// both sensors are getting the same trigger, so these should be nearly equal!
volatile unsigned long start_time_front;
volatile unsigned long start_time_back;

/* ISR to capture the flight time at the front sensor */
void ISR_Front_Sensor()
{
  if (digitalRead(ECHO_PIN_FRONT)) {
    start_time_front = micros();
    state_front = MEASURING;
  }
  else {
    flight_time_front = micros() - start_time_front;
    state_front = READY;
  }
}

/* ISR to capture the flight time at the front sensor */
void ISR_Back_Sensor()
{
  if (digitalRead(ECHO_PIN_BACK)) {
    start_time_back = micros();
    state_back = MEASURING;
  }
  else {
    flight_time_back = micros() - start_time_front;
    state_back = READY;
  }
}

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  // pinMode(TRIGGER_PIN_FRONT, OUTPUT);
  // pinMode(TRIGGER_PIN_BACK, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(DISABLE_WRITE, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_A, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_B, INPUT_PULLUP);

  // get inputs ready
  pinMode(ECHO_PIN_FRONT, INPUT_PULLDOWN);
  pinMode(ECHO_PIN_BACK, INPUT_PULLDOWN);
  // set interrupt modes, link to input pins and ISRs
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FRONT), ISR_Front_Sensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_BACK), ISR_Back_Sensor, CHANGE);

  // We could use higher baudrates!
  Serial.begin(9600);

  // setup SD-Card
  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    sdcard_disabled = true;
    return;
  }
  /* Is the following information (card type and size) important? */
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    sdcard_disabled = true;
    return;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  sdcard_disabled = false;

  // trigger the sensors continuously by generating a pwm signal
  // assumes both sensors are connected to the same trigger pin!
  ledcSetup(CHANNEL, TRIGGER_PWM_FREQUENCY, 8); 
  ledcAttachPin(TRIGGER_PIN, CHANNEL);            
  ledcWrite(CHANNEL, DUTYCYCLE);
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
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

// To analyse the data more preciseley, we want to keep start and end time of all measurements.
// The end time is implicitly given by the calculated distance!
void appendMeasurement(bool ground_truth1, bool ground_truth2) {
  // we don't need this variables to be global
  static bool prev_ground_truth1 = false;
  static bool prev_ground_truth2 = false;
  static uint16_t prev_front_distance = 0;
  static uint16_t prev_back_distance = 0;
  
  // ('flight time' [us] / 1000.0) [ms] * 'speed of sound' [cm/ms] / 2 = distance [cm]
  // roundet and casted to int
  uint16_t front_distance = (uint16_t) (flight_time_front / 1000.0 * 34.3 / 2.0 + 0.5);   
  uint16_t back_distance = (uint16_t) (flight_time_back / 1000.0 * 34.3 / 2.0 + 0.5);
  
  #ifdef DEBUG
    // Prints the distance on the Serial Monitor
    Serial.print(millis());
    Serial.print(",");
    Serial.print(front_distance);
    Serial.print(",");
    Serial.print(back_distance);
    Serial.println();
  #endif

  // we need to save the measurements start time -> no continuous data tracking, only changes are saved
  if (prev_front_distance != front_distance || prev_back_distance != back_distance || prev_ground_truth1 != ground_truth1 || prev_ground_truth2 != ground_truth2) {
    // buffer storing the appended string
    char buf[128];

    sprintf(buf, "%lu, %d, %lu, %d, %d, %d\n", start_time_front, front_distance, start_time_back, back_distance, ground_truth1, ground_truth2);
    appendFile(SD, "/log.csv", buf);

    prev_front_distance = front_distance;
    prev_back_distance = back_distance;
    prev_ground_truth1 = ground_truth1;
    prev_ground_truth2 = ground_truth2;
  }
}

void loop() 
{
  // indicate if the SD-Card can be removed savely
  if (digitalRead(DISABLE_WRITE)) {
    // digitalWrite(GREEN_LED, HIGH);
    Serial.println("Write protection, skipping everything");
    return;
  }
  digitalWrite(GREEN_LED, LOW);

  // both sensors are ready, save the measurements
  if (state_front == READY && state_back == READY) {
    appendMeasurement(digitalRead(GROUND_TRUTH_A), digitalRead(GROUND_TRUTH_B));
    state_back = !READY;
    state_front = !READY;
  }
}