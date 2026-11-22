// STM32U585 (Arduino UNO Q) 펌웨어
// micro-ROS 통신으로 Linux ROS 2와 연결
//
// 필요 라이브러리 (Arduino IDE Library Manager):
//   - micro_ros_arduino  (Micro-ROS Arduino)
//   - Servo              (기본 내장)
//
// 발행 토픽:
//   /imu/raw           (sensor_msgs/Imu)         - 가속도 + 자이로
//   /imu/mag           (sensor_msgs/MagneticField) - 지자기
//   /ultrasonic/range  (sensor_msgs/Range)        - 초음파 거리
//
// 구독 토픽:
//   /servo/angle       (std_msgs/Float32)  - 서보 목표 각도 (0~180°)
//   /led/cmd           (std_msgs/String)   - LED 패턴 명령
//
// 하드웨어 연결:
//   MPU-9250 SDA → A4, SCL → A5
//   HC-SR04 TRIG → D7,  ECHO → D8
//   SG90 서보    → D9

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>
#include <Wire.h>

// ── 핀 정의 ────────────────────────────────────────────
#define TRIG_PIN   7
#define ECHO_PIN   8
#define SERVO_PIN  9

// ── MPU-9250 I2C 주소 ───────────────────────────────────
#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

// ── ROS 엔티티 ──────────────────────────────────────────
rcl_publisher_t     imu_pub, mag_pub, range_pub;
rcl_subscription_t  servo_sub, led_sub;
rclc_executor_t     executor;
rclc_support_t      support;
rcl_allocator_t     allocator;
rcl_node_t          node;
rcl_timer_t         timer;

sensor_msgs__msg__Imu            imu_msg;
sensor_msgs__msg__MagneticField  mag_msg;
sensor_msgs__msg__Range          range_msg;
std_msgs__msg__Float32           servo_cmd_msg;
std_msgs__msg__String            led_cmd_msg;

Servo servo;
float servo_target = 90.0f;

// ── MPU-9250 초기화 ─────────────────────────────────────
void setupMPU9250() {
  Wire.begin();
  // 절전 해제
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  // 가속도 ±4g
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C); Wire.write(0x08);
  Wire.endTransmission();
  // 자이로 ±500dps
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission();
  // AK8963 bypass 활성화
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x37); Wire.write(0x02);
  Wire.endTransmission();
  // AK8963 100Hz 16bit 연속 측정
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A); Wire.write(0x16);
  Wire.endTransmission();
  delay(10);
}

void readAccelGyro(float* ax, float* ay, float* az,
                   float* gx, float* gy, float* gz) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 14);

  // 가속도 (±4g → 8192 LSB/g)
  *ax = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  *ay = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  *az = (int16_t)((Wire.read() << 8) | Wire.read()) / 8192.0f * 9.81f;
  Wire.read(); Wire.read();  // 온도 skip
  // 자이로 (±500dps → 65.5 LSB/dps → rad/s 변환)
  *gx = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
  *gy = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
  *gz = (int16_t)((Wire.read() << 8) | Wire.read()) / 65.5f * 0.01745f;
}

void readMag(float* mx, float* my, float* mz) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 7);
  // AK8963는 little-endian (±4912μT → 32760 LSB)
  *mx = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  *my = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  *mz = (int16_t)((Wire.read()) | (Wire.read() << 8)) / 32760.0f * 4912e-6f;
  Wire.read();  // ST2 (overflow 확인용, 여기선 skip)
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 30000UL);  // 최대 30ms 대기
  return (us == 0) ? 4.0f : us * 0.000017f;   // m 단위
}

// ── LED 매트릭스 패턴 출력 ───────────────────────────────
void updateLED(const char* cmd) {
  // UNO Q LED 매트릭스 또는 외부 LED 제어
  // 아래 주석을 보드 실제 LED 라이브러리에 맞게 수정

  // [UNO R4 호환 LED 매트릭스 사용 시]
  // #include "Arduino_LED_Matrix.h"
  // ArduinoLEDMatrix matrix;
  // if (strcmp(cmd, "DANGER")   == 0) matrix.loadFrame(frame_danger);
  // if (strcmp(cmd, "WARNING")  == 0) matrix.loadFrame(frame_warn);
  // if (strcmp(cmd, "FORWARD")  == 0) matrix.loadFrame(frame_up);
  // ...

  // [임시: 내장 LED로 상태 표시]
  if (strncmp(cmd, "DANGER", 6) == 0) {
    digitalWrite(LED_BUILTIN, HIGH);  // 위험: LED on
  } else if (strncmp(cmd, "WARNING", 7) == 0) {
    // 경고: 토글 (버저 포함 가능)
    digitalWrite(LED_BUILTIN, (millis() / 200) % 2);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// ── micro-ROS 콜백 ──────────────────────────────────────
void timerCb(rcl_timer_t*, int64_t) {
  float ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.angular_velocity.x    = gx;
  imu_msg.angular_velocity.y    = gy;
  imu_msg.angular_velocity.z    = gz;
  imu_msg.orientation_covariance[0] = -1.0;  // orientation 미계산
  rcl_publish(&imu_pub, &imu_msg, NULL);

  float mx, my, mz;
  readMag(&mx, &my, &mz);
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;
  rcl_publish(&mag_pub, &mag_msg, NULL);

  servo.write((int)servo_target);

  range_msg.range = measureDistance();
  rcl_publish(&range_pub, &range_msg, NULL);
}

void servoCb(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  servo_target = constrain(msg->data, 0.0f, 180.0f);
}

void ledCb(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  updateLED(msg->data.data);
}

// ── setup / loop ────────────────────────────────────────
void setup() {
  set_microros_serial_transports(Serial);  // UART로 Linux와 통신
  delay(2000);

  setupMPU9250();
  servo.attach(SERVO_PIN);
  servo.write(90);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "stm32_publisher", "", &support);

  // 발행자
  rclc_publisher_init_default(&imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/raw");
  rclc_publisher_init_default(&mag_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag");
  rclc_publisher_init_default(&range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/range");

  // 구독자
  rclc_subscription_init_default(&servo_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/servo/angle");
  rclc_subscription_init_default(&led_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/led/cmd");

  // 타이머 100Hz
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timerCb);

  // Executor: 타이머 1 + 구독 2 = 3 핸들
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &servo_sub,
    &servo_cmd_msg, servoCb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub,
    &led_cmd_msg, ledCb, ON_NEW_DATA);

  // Range 메시지 초기값
  range_msg.radiation_type = 0;  // ULTRASOUND
  range_msg.field_of_view  = 0.2618f;
  range_msg.min_range      = 0.02f;
  range_msg.max_range      = 4.0f;

  // led_cmd 문자열 버퍼
  led_cmd_msg.data.data     = (char*)malloc(32);
  led_cmd_msg.data.capacity = 32;
  led_cmd_msg.data.size     = 0;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
