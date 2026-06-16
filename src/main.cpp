#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SAMPLE_HZ 50
#define PRINT_HZ 2

#define ENABLE_CALIBRATION 1

Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

bool lsmReady = false;
bool magReady = false;

// --------------------------------------------------
// 중립값
// --------------------------------------------------

float wristNeutralDeg = 0.0f;
float palmNeutralDeg = 0.0f;

// accelStable 판정용 기준값
float accelNeutralTotal = 9.8f;

// --------------------------------------------------
// accelStable 히스테리시스
// --------------------------------------------------
// ENTER: 다시 안정 상태로 들어오는 기준
// EXIT : 안정 상태에서 벗어나는 기준
//
// 예: A0 = 10.17일 때
// 안정 상태에서 A가 9.50 정도로 내려가도 바로 S:N으로 안 바뀜.
// A가 확실히 크게 벗어나야 S:N으로 바뀜.

bool accelStableState = true;

const float ACCEL_STABLE_ENTER_TOL = 0.6f;
const float ACCEL_STABLE_EXIT_TOL  = 0.9f;

// --------------------------------------------------
// 교차 간섭 보정 계수
// --------------------------------------------------

float COUPLE_WRIST_FROM_PALM = 0.0f;
float COUPLE_PALM_FROM_WRIST = 0.0f;

// --------------------------------------------------
// 최종 출력 필터
// --------------------------------------------------

float wristFinalDeg = 0.0f;
float palmFinalDeg = 0.0f;
bool finalAngleInit = false;

const float ALPHA_STABLE = 0.25f;
const float ALPHA_UNSTABLE = 0.04f;
const float ANGLE_DEADBAND_DEG = 0.5f;

// --------------------------------------------------
// 유틸
// --------------------------------------------------

float radToDeg(float rad) {
  return rad * 180.0f / PI;
}

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float lowPassAngleDeg(float currentDeg, float targetDeg, float alpha) {
  float diff = wrap180(targetDeg - currentDeg);
  return wrap180(currentDeg + alpha * diff);
}

float applyDeadbandDeg(float angleDeg, float deadbandDeg) {
  if (fabs(angleDeg) < deadbandDeg) {
    return 0.0f;
  }

  return angleDeg;
}

// --------------------------------------------------
// 센서 초기화
// --------------------------------------------------

bool initLSM6DSOX() {
  if (lsm6dsox.begin_I2C(0x6A, &Wire)) {
    Serial.println("LSM6DSOX OK: 0x6A");
    return true;
  }

  if (lsm6dsox.begin_I2C(0x6B, &Wire)) {
    Serial.println("LSM6DSOX OK: 0x6B");
    return true;
  }

  return false;
}

bool initLIS3MDL() {
  if (lis3mdl.begin_I2C(0x1C, &Wire)) {
    Serial.println("LIS3MDL OK: 0x1C");
    return true;
  }

  if (lis3mdl.begin_I2C(0x1E, &Wire)) {
    Serial.println("LIS3MDL OK: 0x1E");
    return true;
  }

  return false;
}

void readSensors(
  sensors_event_t &accel,
  sensors_event_t &gyro,
  sensors_event_t &temp,
  sensors_event_t &mag
) {
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  if (magReady) {
    lis3mdl.getEvent(&mag);
  }
}

// --------------------------------------------------
// 각도 계산
// --------------------------------------------------

float calcWristFlexDeg(float ax, float ay, float az) {
  return radToDeg(atan2(ay, sqrt(ax * ax + az * az)));
}

float calcPalmFlipSignedDeg(float ax, float ay, float az) {
  float wristRad = atan2(ay, sqrt(ax * ax + az * az));

  float s = sin(wristRad);
  float c = cos(wristRad);

  float azCorrected = ay * s + az * c;

  return radToDeg(atan2(ax, azCorrected));
}

// --------------------------------------------------
// 2축 교차보정
// --------------------------------------------------

void decoupleWristPalm(
  float wristMeasuredDeg,
  float palmMeasuredDeg,
  float &wristOutDeg,
  float &palmOutDeg
) {
  float b = COUPLE_WRIST_FROM_PALM;
  float c = COUPLE_PALM_FROM_WRIST;

  float det = 1.0f - b * c;

  if (fabs(det) < 0.2f) {
    wristOutDeg = wristMeasuredDeg;
    palmOutDeg = palmMeasuredDeg;
    return;
  }

  wristOutDeg = (wristMeasuredDeg - b * palmMeasuredDeg) / det;
  palmOutDeg = (palmMeasuredDeg - c * wristMeasuredDeg) / det;

  wristOutDeg = wrap180(wristOutDeg);
  palmOutDeg = wrap180(palmOutDeg);
}

// --------------------------------------------------
// 중립 캘리브레이션
// --------------------------------------------------

void calibrateNeutralPose() {
  Serial.println("CAL START: keep neutral pose");

  delay(1000);

  const int samples = SAMPLE_HZ * 3;

  float axSum = 0.0f;
  float aySum = 0.0f;
  float azSum = 0.0f;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;

  for (int i = 0; i < samples; i++) {
    readSensors(accel, gyro, temp, mag);

    axSum += accel.acceleration.x;
    aySum += accel.acceleration.y;
    azSum += accel.acceleration.z;

    delay(1000 / SAMPLE_HZ);
  }

  float axAvg = axSum / samples;
  float ayAvg = aySum / samples;
  float azAvg = azSum / samples;

  wristNeutralDeg = calcWristFlexDeg(axAvg, ayAvg, azAvg);
  palmNeutralDeg = calcPalmFlipSignedDeg(axAvg, ayAvg, azAvg);

  accelNeutralTotal = sqrt(axAvg * axAvg + ayAvg * ayAvg + azAvg * azAvg);

  accelStableState = true;

  Serial.print("CAL DONE | W0:");
  Serial.print(wristNeutralDeg, 2);

  Serial.print(" P0:");
  Serial.print(palmNeutralDeg, 2);

  Serial.print(" A0:");
  Serial.print(accelNeutralTotal, 2);

  Serial.print(" ENTER:");
  Serial.print(ACCEL_STABLE_ENTER_TOL, 2);

  Serial.print(" EXIT:");
  Serial.println(ACCEL_STABLE_EXIT_TOL, 2);
}

// --------------------------------------------------
// setup
// --------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.setClock(100000);
  Wire.setTimeOut(50);

  Serial.println("==== CTS WRIST IMU TEST ====");

  lsmReady = initLSM6DSOX();

  if (!lsmReady) {
    Serial.println("ERROR: LSM6DSOX not found");
    while (true) {
      delay(1000);
    }
  }

  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  magReady = initLIS3MDL();

  if (magReady) {
    lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  } else {
    Serial.println("LIS3MDL SKIP");
  }

#if ENABLE_CALIBRATION
  calibrateNeutralPose();
#else
  wristNeutralDeg = 0.0f;
  palmNeutralDeg = 0.0f;
  accelNeutralTotal = 9.8f;
  accelStableState = true;

  Serial.println("CAL SKIP");
#endif

  Serial.println("START");
}

// --------------------------------------------------
// loop
// --------------------------------------------------

void loop() {
  static uint32_t lastSampleMs = 0;
  static uint32_t lastPrintMs = 0;

  uint32_t nowMs = millis();

  if (nowMs - lastSampleMs < 1000 / SAMPLE_HZ) {
    return;
  }

  lastSampleMs = nowMs;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;

  readSensors(accel, gyro, temp, mag);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float accelTotal = sqrt(ax * ax + ay * ay + az * az);
  float accelErr = fabs(accelTotal - accelNeutralTotal);

  // --------------------------------------------------
  // accelStable 히스테리시스 판정
  // --------------------------------------------------

  if (accelStableState) {
    if (accelErr > ACCEL_STABLE_EXIT_TOL) {
      accelStableState = false;
    }
  } else {
    if (accelErr < ACCEL_STABLE_ENTER_TOL) {
      accelStableState = true;
    }
  }

  bool accelStable = accelStableState;

  float alphaUse = accelStable ? ALPHA_STABLE : ALPHA_UNSTABLE;

  // --------------------------------------------------
  // 각도 후보값 계산
  // --------------------------------------------------

  float wristMeasuredRawDeg = calcWristFlexDeg(ax, ay, az);
  float palmMeasuredRawDeg = calcPalmFlipSignedDeg(ax, ay, az);

  float wristMeasuredDeg = wrap180(wristMeasuredRawDeg - wristNeutralDeg);
  float palmMeasuredDeg = wrap180(palmMeasuredRawDeg - palmNeutralDeg);

  // --------------------------------------------------
  // 교차보정
  // --------------------------------------------------

  float wristDecoupledDeg = 0.0f;
  float palmDecoupledDeg = 0.0f;

  decoupleWristPalm(
    wristMeasuredDeg,
    palmMeasuredDeg,
    wristDecoupledDeg,
    palmDecoupledDeg
  );

  // --------------------------------------------------
  // 최종 smoothing
  // --------------------------------------------------

  if (!finalAngleInit) {
    wristFinalDeg = wristDecoupledDeg;
    palmFinalDeg = palmDecoupledDeg;
    finalAngleInit = true;
  } else {
    wristFinalDeg = lowPassAngleDeg(wristFinalDeg, wristDecoupledDeg, alphaUse);
    palmFinalDeg = lowPassAngleDeg(palmFinalDeg, palmDecoupledDeg, alphaUse);
  }

  wristFinalDeg = applyDeadbandDeg(wristFinalDeg, ANGLE_DEADBAND_DEG);
  palmFinalDeg = applyDeadbandDeg(palmFinalDeg, ANGLE_DEADBAND_DEG);

  // --------------------------------------------------
  // 출력
  // --------------------------------------------------

  if (nowMs - lastPrintMs >= 1000 / PRINT_HZ) {
    lastPrintMs = nowMs;

    Serial.print("FINAL W:");
    Serial.print(wristFinalDeg, 2);

    Serial.print(" P:");
    Serial.print(palmFinalDeg, 2);

    Serial.print(" | RAW W:");
    Serial.print(wristDecoupledDeg, 2);

    Serial.print(" P:");
    Serial.print(palmDecoupledDeg, 2);

    Serial.print(" | A:");
    Serial.print(accelTotal, 2);

    Serial.print(" A0:");
    Serial.print(accelNeutralTotal, 2);

    Serial.print(" E:");
    Serial.print(accelErr, 2);

    Serial.print(" | S:");
    Serial.print(accelStable ? "Y" : "N");

    Serial.print(" | F:");
    Serial.println(alphaUse, 2);
  }
}