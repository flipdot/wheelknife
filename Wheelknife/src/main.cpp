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
int front_distance;
int back_distance;
int last_front_distance;
int last_back_distance;
int last_ground_truth1;
int last_ground_truth2;
MovingAverage front_ma = MovingAverage(10);
MovingAverage back_ma = MovingAverage(10);

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(FRONT_SENSOR_TRIGGER, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(BACK_SENSOR_TRIGGER, OUTPUT);
  pinMode(BACK_SENSOR_ECHO, INPUT);
  pinMode(DISABLE_WRITE, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_A, INPUT_PULLUP);
  pinMode(GROUND_TRUTH_B, INPUT_PULLUP);
  Serial.begin(9600);
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
  sprintf(buf, "%d,%d,%d,%d,%d,%d,%d\n", start_time,mid_time,end_time, front_distance,back_distance ground_truth1,ground_truth2);

  //if (last_front_distance != front_distance || last_back_distance != back_distance || last_ground_truth1 != ground_truth1 || last_ground_truth2 != ground_truth2) {
    appendFile(SD, "/log.csv", chr_buf);
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

void loop() {
  int disable_write = digitalRead(DISABLE_WRITE);
  if (disable_write) {
    digitalWrite(GREEN_LED, HIGH);
    return;
  }
  digitalWrite(GREEN_LED, LOW);
  
  int startTime = millis();
  front_distance = get_distance(FRONT_SENSOR_TRIGGER, FRONT_SENSOR_ECHO);
  int midTime = millis();
  back_distance = get_distance(BACK_SENSOR_TRIGGER, BACK_SENSOR_ECHO);
  int endTime = millis();

  appendMeasurement(startTime, midTime, endTime, front_distance, back_distance, digitalRead(GROUND_TRUTH_A), digitalRead(GROUND_TRUTH_B));

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

