/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/faq/how-to-control-speed-of-servo-motor
 */


#include <Servo.h>

// 핀 설정
const int PIN_SERVO = 10;
const int PIN_LED   = 9;
const int PIN_TRIG  = 12;
const int PIN_ECHO  = 13;

// 차단기 각도 설정
const int ANGLE_CLOSED = 0;    // 차단기 닫힘
const int ANGLE_OPEN   = 100;  // 차단기 열림

// 초음파 센서 설정
const float DETECT_DISTANCE_CM = 40.0;
const float RELEASE_DISTANCE_CM = 50.0;

const unsigned long SENSOR_INTERVAL_MS = 60UL;
const unsigned long ECHO_TIMEOUT_US = 25000UL;

// 차단기 동작 설정
const unsigned long FULL_TRAVEL_MS = 1500UL;
const unsigned long CLEAR_DELAY_MS = 3000UL;

// 서보모터 객체
Servo barrierServo;

// 현재 서보 각도와 이동 정보
float currentAngle = ANGLE_CLOSED;
float movementStartAngle = ANGLE_CLOSED;

int targetAngle = ANGLE_CLOSED;

unsigned long movementStartMs = 0;
unsigned long movementDurationMs = 0;

bool moving = false;

// 센서 및 차량 감지 상태
unsigned long lastSensorReadMs = 0;
unsigned long lastDetectedMs = 0;

bool sensorSeesObject = false;
bool vehicleWasDetected = false;


// 초음파 센서로 거리를 측정
// 정상적으로 측정하지 못하면 -1 반환
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration =
      pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0;
  }

  // 음속 약 0.0343 cm/us, 왕복 거리이므로 2로 나눔
  return duration * 0.0343 / 2.0;
}


// Smoothstep 보간을 이용한 서보모터 이동 시작
void beginMovement(int newTarget, unsigned long now) {
  if (newTarget == targetAngle) {
    return;
  }

  targetAngle = newTarget;

  // 현재 위치에서 새 목표 위치로 이동
  movementStartAngle = currentAngle;
  movementStartMs = now;

  float distance = targetAngle - currentAngle;
  if (distance < 0) {
    distance = -distance;
  }

  int fullRange = ANGLE_OPEN - ANGLE_CLOSED;
  if (fullRange < 0) {
    fullRange = -fullRange;
  }

  if (fullRange == 0 || distance < 0.5) {
    currentAngle = targetAngle;
    moving = false;
    barrierServo.write(targetAngle);
    return;
  }

  // 이동 거리에 비례해 이동 시간 설정
  movementDurationMs =
      (unsigned long)(
          (float)FULL_TRAVEL_MS * distance / fullRange
      );

  if (movementDurationMs < 1UL) {
    movementDurationMs = 1UL;
  }

  moving = true;
}


// 서보모터의 현재 위치를 시간에 따라 갱신
void updateServoMotion(unsigned long now) {
  if (!moving) {
    return;
  }

  unsigned long elapsed = now - movementStartMs;

  if (movementDurationMs == 0 ||
      elapsed >= movementDurationMs) {

    currentAngle = targetAngle;
    moving = false;

  } else {
    // 진행률을 0~1 범위로 정규화
    float t = (float)elapsed / movementDurationMs;

    // Smoothstep: sigmoid가 아닌 3차 다항식
    float eased = t * t * (3.0 - 2.0 * t);

    currentAngle =
        movementStartAngle +
        (targetAngle - movementStartAngle) * eased;
  }

  // 서보 라이브러리는 정수 각도를 받음
  barrierServo.write((int)(currentAngle + 0.5));
}


void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(9600);

  barrierServo.attach(PIN_SERVO);

  // 시작할 때 차단기가 닫힌 상태라고 가정
  currentAngle = ANGLE_CLOSED;
  movementStartAngle = ANGLE_CLOSED;
  targetAngle = ANGLE_CLOSED;

  barrierServo.write(ANGLE_CLOSED);

  lastSensorReadMs = millis();
}


void loop() {
  unsigned long now = millis();

  // 센서를 측정하지 않는 동안에도 모터 움직임 갱신
  updateServoMotion(now);

  // 일정한 간격으로 초음파 센서 측정
  if (now - lastSensorReadMs >= SENSOR_INTERVAL_MS) {
    lastSensorReadMs = now;

    float distance = readDistanceCm();

    if (distance < 0) {
      // 측정 실패: 감지 상태를 해제하되,
      // 기존 감지 기록을 바로 삭제하지는 않음
      sensorSeesObject = false;

    } else {
      // 감지 시작 거리와 해제 거리를 다르게 설정
      // 하여 센서값이 조금 흔들려도 상태가 반복되지 않게 함
      if (!sensorSeesObject &&
          distance <= DETECT_DISTANCE_CM) {

        sensorSeesObject = true;

      } else if (sensorSeesObject &&
                 distance >= RELEASE_DISTANCE_CM) {

        sensorSeesObject = false;
      }
    }

    if (sensorSeesObject) {
      lastDetectedMs = millis();
      vehicleWasDetected = true;
    }

    // 시리얼 모니터에서 거리 확인
    Serial.print("Distance: ");

    if (distance < 0) {
      Serial.println("No echo");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
  }

  now = millis();

  // 물체가 감지되고 있거나,
  // 마지막 감지 후 일정 시간이 지나지 않았다면 열림 유지
  bool shouldOpen =
      vehicleWasDetected &&
      (sensorSeesObject ||
       now - lastDetectedMs < CLEAR_DELAY_MS);

  if (vehicleWasDetected && !shouldOpen) {
    vehicleWasDetected = false;
  }

  int desiredTarget =
      shouldOpen ? ANGLE_OPEN : ANGLE_CLOSED;

  // 목표 각도가 바뀌었을 때만 이동 방향 전환
  if (desiredTarget != targetAngle) {
    updateServoMotion(now);
    beginMovement(desiredTarget, now);
  }

  // 모터가 움직이는 동안에도 계속 각도 갱신
  updateServoMotion(millis());

  // LED는 물체 감지에 따른 차단기 동작 중에 켜짐
  bool barrierActive =
      shouldOpen ||
      moving ||
      currentAngle > ANGLE_CLOSED + 0.5;

  digitalWrite(PIN_LED, barrierActive ? HIGH : LOW);
}
