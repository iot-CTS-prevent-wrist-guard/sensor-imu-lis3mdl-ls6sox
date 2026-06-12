#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SAMPLE_HZ 50
#define PRINT_HZ 10

#define ENABLE_CALIBRATION 1

Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

bool lsmReady = false;
bool magReady = false;

// --------------------------------------------------
// 최종 각도 중립값
// --------------------------------------------------

float wristNeutralDeg = 0.0f;
float palmNeutralDeg = 0.0f;

// 비교용 TILT palm 중립값
float palmTiltNeutralDeg = 0.0f;

// --------------------------------------------------
// 교차 간섭 보정 계수
// --------------------------------------------------
// b: palm 움직임이 wrist 값에 섞이는 비율
// c: wrist 움직임이 palm 값에 섞이는 비율
//
// 처음에는 둘 다 0으로 둔다.
// 로그 보고 나중에 조정한다.
//
// 예:
// wrist만 움직였는데 wrist=40, palm=8이면
// c = 8 / 40 = 0.2
//
// palm만 움직였는데 palm=50, wrist=-10이면
// b = -10 / 50 = -0.2

float COUPLE_WRIST_FROM_PALM = 0.0f;  // b
float COUPLE_PALM_FROM_WRIST = 0.0f;  // c

// --------------------------------------------------
// 최종 출력 필터
// --------------------------------------------------

float wristFinalDeg = 0.0f;
float palmFinalDeg = 0.0f;
bool finalAngleInit = false;

const float ALPHA_FINAL = 0.25f;        // 클수록 빠르게 반응
const float ANGLE_DEADBAND_DEG = 0.5f;  // 이 이하 각도는 0 처리

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
    Serial.println("LSM6DSOX found at 0x6A");
    return true;
  }

  if (lsm6dsox.begin_I2C(0x6B, &Wire)) {
    Serial.println("LSM6DSOX found at 0x6B");
    return true;
  }

  return false;
}

bool initLIS3MDL() {
  if (lis3mdl.begin_I2C(0x1C, &Wire)) {
    Serial.println("LIS3MDL found at 0x1C");
    return true;
  }

  if (lis3mdl.begin_I2C(0x1E, &Wire)) {
    Serial.println("LIS3MDL found at 0x1E");
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
// 기존 RAW 비교용
// --------------------------------------------------

float calcWristRawDeg(float ay, float az) {
  return radToDeg(atan2(ay, az));
}

float calcPalmRawDeg(float ax, float az) {
  return radToDeg(atan2(ax, az));
}

// --------------------------------------------------
// wrist 계산
// --------------------------------------------------
// 손목 굽힘/젖힘.
// 네 장착 기준에서 Y-Z 변화로 보는 값.
// palm 회전 영향을 줄이려고 기준축을 az 하나가 아니라
// sqrt(ax^2 + az^2)로 잡는다.

float calcWristFlexDeg(float ax, float ay, float az) {
  return radToDeg(atan2(ay, sqrt(ax * ax + az * az)));
}

// --------------------------------------------------
// 기존 palm TILT 비교용
// --------------------------------------------------
// 이 방식은 wrist 영향은 줄지만,
// sqrt 때문에 90도 이후 방향이 접히는 문제가 있음.
// 최종 palm 판정용으로 쓰지 말고 비교용으로만 본다.

float calcPalmFlipTiltDeg(float ax, float ay, float az) {
  return radToDeg(atan2(ax, sqrt(ay * ay + az * az)));
}

// --------------------------------------------------
// 최종 palm signed 계산
// --------------------------------------------------
// 손바닥 뒤집힘.
// 기존 TILT palm은 sqrt 때문에 부호가 접힌다.
// 그래서 wrist 각도를 먼저 추정한 뒤,
// wrist 굽힘 때문에 바뀐 Z축을 보정하고,
// ax와 보정된 az로 atan2를 계산한다.
//
// 이 값이 최종 palm 후보값이다.

float calcPalmFlipSignedDeg(float ax, float ay, float az) {
  float wristRad = atan2(ay, sqrt(ax * ax + az * az));

  float s = sin(wristRad);
  float c = cos(wristRad);

  // wrist 굽힘 성분을 반영해서 Z축 기준을 보정
  float azCorrected = ay * s + az * c;

  return radToDeg(atan2(ax, azCorrected));
}

// --------------------------------------------------
// 2축 교차보정
// --------------------------------------------------
// wristMeasured = wristTrue + b * palmTrue
// palmMeasured  = c * wristTrue + palmTrue
//
// 역행렬로 wristTrue, palmTrue를 추정한다.
// 이 방식은 한쪽을 고정하지 않는다.
// wrist와 palm이 동시에 움직여도 둘 다 계산한다.

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
    // 보정식 폭주 방지
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
  Serial.println();
  Serial.println("Neutral calibration start");
  Serial.println("Keep the hand in neutral position for 3 seconds.");

  delay(1000);

  const int samples = 150;

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

  palmTiltNeutralDeg = calcPalmFlipTiltDeg(axAvg, ayAvg, azAvg);

  Serial.println("Neutral calibration done");

  Serial.print("neutral accel avg | X: ");
  Serial.print(axAvg, 4);
  Serial.print(" Y: ");
  Serial.print(ayAvg, 4);
  Serial.print(" Z: ");
  Serial.println(azAvg, 4);

  Serial.print("neutral final deg | wrist: ");
  Serial.print(wristNeutralDeg, 3);
  Serial.print(" palm_signed: ");
  Serial.println(palmNeutralDeg, 3);

  Serial.print("neutral compare   | palm_tilt: ");
  Serial.println(palmTiltNeutralDeg, 3);

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  // 네 기존 안정 코드 기준 유지.
  // 400kHz로 올리지 마라. I2C Error -1 다시 날 수 있음.
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  Serial.println();
  Serial.println("==== CTS WRIST IMU DECOUPLED ANGLE TEST ====");
  Serial.println("-Y axis = finger direction");
  Serial.println("+Y axis = wrist direction");
  Serial.println("Sensor position = back of hand");
  Serial.println("AHRS removed");
  Serial.println("FINAL wrist = atan2(ay, sqrt(ax^2 + az^2))");
  Serial.println("FINAL palm  = atan2(ax, az_corrected)");
  Serial.println("DECOUPLING  = 2x2 cross-coupling correction");
  Serial.println();

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

  Serial.println("LSM6DSOX configured");

  magReady = initLIS3MDL();

  if (magReady) {
    lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    Serial.println("LIS3MDL configured");
  } else {
    Serial.println("WARNING: LIS3MDL not found. Continue with accel/gyro only.");
  }

#if ENABLE_CALIBRATION
  calibrateNeutralPose();
#else
  wristNeutralDeg = 0.0f;
  palmNeutralDeg = 0.0f;
  palmTiltNeutralDeg = 0.0f;

  Serial.println();
  Serial.println("Neutral calibration skipped");
  Serial.println();
#endif

  Serial.println("Start angle output");
}

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

  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;

  if (magReady) {
    mx = mag.magnetic.x;
    my = mag.magnetic.y;
    mz = mag.magnetic.z;
  }

  float accelTotal = sqrt(ax * ax + ay * ay + az * az);
  float magTotal = sqrt(mx * mx + my * my + mz * mz);

  // 빠른 움직임이면 가속도 기반 각도는 순간적으로 흔들릴 수 있음.
  bool accelStable = accelTotal > 8.5f && accelTotal < 11.5f;

  // --------------------------------------------------
  // 원시 비교용 각도
  // --------------------------------------------------

  float wristRawDeg = calcWristRawDeg(ay, az);
  float palmRawDeg = calcPalmRawDeg(ax, az);

  // --------------------------------------------------
  // 기존 TILT 비교용
  // --------------------------------------------------

  float wristTiltRawDeg = calcWristFlexDeg(ax, ay, az);
  float palmTiltRawDeg = calcPalmFlipTiltDeg(ax, ay, az);

  float wristTiltDeg = wrap180(wristTiltRawDeg - wristNeutralDeg);
  float palmTiltDeg = wrap180(palmTiltRawDeg - palmTiltNeutralDeg);

  // --------------------------------------------------
  // 최종 후보값
  // --------------------------------------------------
  // wrist: TILT 방식
  // palm: signed corrected 방식
  //
  // 이 둘은 아직 교차보정 전 값이다.

  float wristMeasuredRawDeg = calcWristFlexDeg(ax, ay, az);
  float palmMeasuredRawDeg = calcPalmFlipSignedDeg(ax, ay, az);

  float wristMeasuredDeg = wrap180(wristMeasuredRawDeg - wristNeutralDeg);
  float palmMeasuredDeg = wrap180(palmMeasuredRawDeg - palmNeutralDeg);

  // --------------------------------------------------
  // 2축 교차보정
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
  // 최종 출력 smoothing
  // --------------------------------------------------

  if (!finalAngleInit) {
    wristFinalDeg = wristDecoupledDeg;
    palmFinalDeg = palmDecoupledDeg;
    finalAngleInit = true;
  } else {
    wristFinalDeg = lowPassAngleDeg(wristFinalDeg, wristDecoupledDeg, ALPHA_FINAL);
    palmFinalDeg = lowPassAngleDeg(palmFinalDeg, palmDecoupledDeg, ALPHA_FINAL);
  }

  wristFinalDeg = applyDeadbandDeg(wristFinalDeg, ANGLE_DEADBAND_DEG);
  palmFinalDeg = applyDeadbandDeg(palmFinalDeg, ANGLE_DEADBAND_DEG);

  if (nowMs - lastPrintMs >= 1000 / PRINT_HZ) {
    lastPrintMs = nowMs;

    Serial.println("----------------------------------------");

    // --------------------------------------------------
    // 1. ACCEL
    // --------------------------------------------------
    // TOTAL이 9.8 근처면 정지/저속.
    // STABLE NO면 순간 각도는 너무 믿지 마라.
    Serial.print("ACCEL m/s^2 | X: ");
    Serial.print(ax, 4);
    Serial.print(" Y: ");
    Serial.print(ay, 4);
    Serial.print(" Z: ");
    Serial.print(az, 4);
    Serial.print(" | TOTAL: ");
    Serial.println(accelTotal, 4);

    Serial.print("STABLE      | ");
    Serial.println(accelStable ? "YES" : "NO");

    // --------------------------------------------------
    // 2. GYRO
    // --------------------------------------------------
    // 지금 최종 각도 계산에는 안 씀.
    // 움직임이 빠른지 확인하는 참고값.
    Serial.print("GYRO rad/s  | X: ");
    Serial.print(gx, 4);
    Serial.print(" Y: ");
    Serial.print(gy, 4);
    Serial.print(" Z: ");
    Serial.println(gz, 4);

    // --------------------------------------------------
    // 3. MAG
    // --------------------------------------------------
    // AHRS 제거했으므로 최종 계산에는 안 씀.
    // 자기계 상태 확인용 출력만 유지.
    if (magReady) {
      Serial.print("MAG uT      | X: ");
      Serial.print(mx, 4);
      Serial.print(" Y: ");
      Serial.print(my, 4);
      Serial.print(" Z: ");
      Serial.print(mz, 4);
      Serial.print(" | TOTAL: ");
      Serial.println(magTotal, 4);
    }

    // --------------------------------------------------
    // 4. 최종값
    // --------------------------------------------------
    // 지금 제일 봐야 하는 줄.
    // 앱/알림/판정에 쓸 후보값.
    //
    // wrist_flex:
    //   손목 굽힘/젖힘
    //
    // palm_flip:
    //   손바닥 뒤집힘
    //
    // 이 값은 둘 다 동시에 업데이트된다.
    // 한쪽 움직인다고 반대쪽을 강제로 고정하지 않는다.
    Serial.print("ANGLE FINAL | wrist_flex: ");
    Serial.print(wristFinalDeg, 3);
    Serial.print(" | palm_flip: ");
    Serial.println(palmFinalDeg, 3);

    // --------------------------------------------------
    // 5. 교차보정 전/후 비교
    // --------------------------------------------------
    // measured:
    //   공식으로 바로 계산한 값
    //
    // decoupled:
    //   COUPLE 계수로 교차 간섭을 뺀 값
    //
    // 처음에는 COUPLE이 0이라 measured와 decoupled가 거의 같다.
    Serial.print("DECOUP RAW  | wrist_measured: ");
    Serial.print(wristMeasuredDeg, 3);
    Serial.print(" | palm_measured: ");
    Serial.print(palmMeasuredDeg, 3);
    Serial.print(" | wrist_decoupled: ");
    Serial.print(wristDecoupledDeg, 3);
    Serial.print(" | palm_decoupled: ");
    Serial.println(palmDecoupledDeg, 3);

    // --------------------------------------------------
    // 6. 기존 TILT 비교용
    // --------------------------------------------------
    // 기존 방식이 얼마나 튀는지 비교하는 줄.
    // 최종 판단은 ANGLE FINAL을 봐라.
    Serial.print("ATAN2 TILT  | wrist_flex: ");
    Serial.print(wristTiltDeg, 3);
    Serial.print(" | palm_flip: ");
    Serial.println(palmTiltDeg, 3);

    // --------------------------------------------------
    // 7. RAW 비교용
    // --------------------------------------------------
    // 문제 원인 확인용.
    // 최종값으로 쓰지 마라.
    Serial.print("ATAN2 RAW   | wrist_raw: ");
    Serial.print(wristRawDeg, 3);
    Serial.print(" | palm_raw: ");
    Serial.println(palmRawDeg, 3);

    // --------------------------------------------------
    // 8. 보정 계수 확인
    // --------------------------------------------------
    Serial.print("COUPLE      | wrist_from_palm: ");
    Serial.print(COUPLE_WRIST_FROM_PALM, 3);
    Serial.print(" | palm_from_wrist: ");
    Serial.println(COUPLE_PALM_FROM_WRIST, 3);

    Serial.print("TEMP C      | ");
    Serial.println(temp.temperature, 2);
  }
}