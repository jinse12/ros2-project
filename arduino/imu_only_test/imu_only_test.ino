// IMU Serial 송신 스케치 (micro_ros_arduino 불필요)
//
// 하드웨어 연결:
//   MPU-9250  VCC → 3.3V
//             GND → GND
//             SCL → A5
//             SDA → A4
//
// 출력 포맷 (115200 baud):
//   ax,ay,az,gx,gy,gz,mx,my,mz
//   단위: m/s², rad/s, Tesla

#include <Wire.h>

#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

void setupMPU9250() {
  Wire.begin();
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); Wire.write(0x00);  // 절전 해제
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C); Wire.write(0x08);  // 가속도 ±4g
  Wire.endTransmission();
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B); Wire.write(0x08);  // 자이로 ±500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x37); Wire.write(0x02);  // AK8963 bypass
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A); Wire.write(0x16);  // 100Hz 16bit 연속
  Wire.endTransmission();
  delay(10);
}

void readAccelGyro(float* ax, float* ay, float* az,
                   float* gx, float* gy, float* gz) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 14);
  *ax = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  *ay = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  *az = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  Wire.read(); Wire.read();  // 온도 skip
  *gx = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
  *gy = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
  *gz = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
}

void readMag(float* mx, float* my, float* mz) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 7);
  *mx = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  *my = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  *mz = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  Wire.read();  // ST2 skip
}

void setup() {
  Serial.begin(115200);
  setupMPU9250();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // 준비 완료
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
  readMag(&mx, &my, &mz);

  // CSV 한 줄 출력
  Serial.print(ax, 4); Serial.print(',');
  Serial.print(ay, 4); Serial.print(',');
  Serial.print(az, 4); Serial.print(',');
  Serial.print(gx, 4); Serial.print(',');
  Serial.print(gy, 4); Serial.print(',');
  Serial.print(gz, 4); Serial.print(',');
  Serial.print(mx, 7); Serial.print(',');
  Serial.print(my, 7); Serial.print(',');
  Serial.println(mz, 7);

  delay(10);  // 100Hz
}
