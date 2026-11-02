#include <Servo.h>
#define PIN_SERVO 10
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO

// configurable parameters for Servo
#define _DUTY_MIN 500 // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2500 // servo full counter-clockwise position (180 degree)

#define _POS_START _DUTY_MIN    // servo start position
#define _POS_END   _DUTY_MAX   // servo end position

#define _SERVO_SPEED 3 // servo angular speed (unit: degree/sec)

// Loop Interval
#define INTERVAL 20     // servo update interval (unit: msec)

// global variables
unsigned long last_sampling_time; // unit: msec

Servo myservo;

float duty_change_per_interval; // maximum duty difference per interval
float duty_target;    // Target duty time
float duty_curr;      // Current duty time


void setup() {
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  
  duty_curr = _POS_START;
  duty_target = _POS_END;
  myservo.writeMicroseconds(duty_curr);
  
  // initialize serial port
  Serial.begin(57600);  // <----- baud rate

  // convert angular velocity into duty change per interval.
  // duty_change_per_interval = 
  //  (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180) * (INTERVAL / 1000);
  duty_change_per_interval =
    (_DUTY_MAX - _DUTY_MIN)
    * (_SERVO_SPEED / 180.0)
    * (INTERVAL / 1000.0);
  
  // initialize last sampling time
  last_sampling_time = 0;
  
}

void loop() {
  float  dist_raw;

  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  // adjust duty_curr toward duty_target by duty_change_per_interval
  if (duty_target > duty_curr) {
    duty_curr += duty_change_per_interval;
    if (duty_curr > duty_target)
        duty_curr = duty_target;
  } else {
    duty_curr -= duty_change_per_interval;
    if (duty_curr < duty_target)
      duty_curr = duty_target;
  }

  // update servo position
  myservo.writeMicroseconds((int)duty_curr);

  // update last sampling time
  last_sampling_time += INTERVAL;
}
