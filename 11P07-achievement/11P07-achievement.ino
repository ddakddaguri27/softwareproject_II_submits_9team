#include <Servo.h>

// 핀 설정
#define PIN_LED   9
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// 초음파 센서 설정
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10

// 거리 단위: mm
#define _DIST_MIN 180   // 18cm
#define _DIST_MAX 360   // 36cm

#define TIMEOUT_US 12000UL
#define SCALE (0.001 * 0.5 * SND_VEL)

// EMA 필터
#define _EMA_ALPHA 0.7

Servo myservo;

unsigned long last_sampling_time = 0;

// 초기 거리: 27cm, 이에 대응하는 서보 각도는 90도
float dist_ema = 270.0;
bool ema_initialized = false;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, LOW);

  myservo.attach(PIN_SERVO);
  myservo.write(90);

  Serial.begin(57600);
}

void loop() {
  // 일정한 간격으로 센서 측정
  if (millis() - last_sampling_time < INTERVAL)
    return;

  last_sampling_time = millis();

  // 초음파 센서 측정값(mm)
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 실제 측정값이 18~36cm 범위에 있는지 확인
  bool in_range =
      (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);

  // 측정에 성공한 경우에만 필터 갱신
  if (dist_raw > 0.0) {

    // 범위 필터: 서보 제어에 사용할 거리를 180~360mm로 제한
    float dist_limited =
        constrain(dist_raw, (float)_DIST_MIN, (float)_DIST_MAX);

    // 최초 측정은 즉시 반영하고, 이후부터 EMA 적용
    if (!ema_initialized) {
      dist_ema = dist_limited;
      ema_initialized = true;
    } else {
      dist_ema =
          _EMA_ALPHA * dist_limited
          + (1.0 - _EMA_ALPHA) * dist_ema;
    }
  }

  // 측정 실패(dist_raw == 0) 시 EMA를 갱신하지 않음
  // 따라서 측정 실패만으로 서보가 갑자기 0도로 이동하지 않음

  // 거리 180~360mm를 서보 각도 0~180도로 선형 변환
  float angle_f =
      (dist_ema - _DIST_MIN) * 180.0
      / (_DIST_MAX - _DIST_MIN);

  int servo_angle = constrain((int)(angle_f + 0.5), 0, 180);

  myservo.write(servo_angle);

  // 실제 측정 거리가 18~36cm일 때 LED 점등
  digitalWrite(PIN_LED, in_range ? LOW : HIGH);

  // Serial Plotter 출력
  Serial.print("Min:");
  Serial.print(_DIST_MIN);

  Serial.print(",dist:");
  Serial.print(dist_raw);

  Serial.print(",ema:");
  Serial.print(dist_ema);

  Serial.print(",Servo:");
  Serial.print(myservo.read());

  Serial.print(",Max:");
  Serial.print(_DIST_MAX);

  Serial.println("");
}

// 초음파 측정 함수
// 반환값: 거리(mm), 측정 실패 시 0
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);

  digitalWrite(TRIG, LOW);

  unsigned long duration =
      pulseIn(ECHO, HIGH, TIMEOUT_US);

  if (duration == 0)
    return 0.0;

  return duration * SCALE;
}
