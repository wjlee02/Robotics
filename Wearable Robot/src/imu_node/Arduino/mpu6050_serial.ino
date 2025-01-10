#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu(0x68);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // 초기 메시지 제거
  if (!mpu.testConnection()) {
    while (1); // 연결 실패 시 멈춤
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  // 가속도 및 자이로 데이터 읽기
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // 데이터를 CSV 형식으로 출력
  Serial.print(ax);  Serial.print(","); // X축 가속도
  Serial.print(ay);  Serial.print(","); // Y축 가속도
  Serial.print(az);  Serial.print(","); // Z축 가속도
  Serial.print(gx);  Serial.print(","); // X축 자이로스코프 각속도
  Serial.print(gy);  Serial.print(","); // Y축 자이로스코프 각속도
  Serial.println(gz); // Z축 자이로스코프 각속도

  delay(500); // 500ms 간격
}
