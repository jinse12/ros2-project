# 기말 프로젝트 주제 선정 보고서

## 1. 작품명

**IMU 기반 자율 장애물 회피 시스템**
— Arduino UNO Q + MPU-9250 + ROS 2 Turtlesim

---

## 2. 작품설명

사용자가 Arduino UNO Q 보드를 손으로 기울이면, 보드에 장착된 9축 IMU 센서(MPU-9250)가 기울기(Roll/Pitch/Yaw)를 측정하여 ROS 2 위에서 동작하는 가상 거북이(turtlesim)를 실시간으로 조종한다.

서보모터에 장착된 초음파 센서(HC-SR04)는 거북이의 진행 방향을 자동으로 추적하며 전방 장애물 거리를 측정한다. 장애물이 감지되면 ROS 2 노드가 자율적으로 회피 기동을 수행한다.

```
[사용자] 보드 기울기
    ↓
[STM32] MPU-9250 I2C 측정 → Serial 전송
    ↓
[WSL2 / QRB2210 Linux]
  serial_imu_bridge → /imu/raw, /imu/mag
  imu_filter_node   → /imu/filtered  (상보 필터)
  obstacle_avoider  → /turtle1/cmd_vel (회피 로직)
    ↓
[turtlesim] 거북이 이동 + 장애물 자율 회피
    ↓
[App Lab 대시보드] 실시간 센서 모니터링 (브라우저)
```

**주요 기능**
| 기능 | 내용 |
|---|---|
| IMU 조종 | 보드 기울기로 turtle 전진/후진/좌회전/우회전 |
| 상보 필터 | Accel(98%) + Gyro(2%) 융합으로 안정적인 각도 추정 |
| 서보 추적 | turtle 진행 방향으로 초음파 센서 자동 조준 |
| 자율 회피 | 0.3m 이내 위험 → 후진+회전, 0.6m 이내 경고 → 감속+편향 |
| 대시보드 | rosbridge + roslibjs 기반 실시간 웹 모니터링 |

---

## 3. 선정이유

**Arduino UNO Q를 선택한 기술적 근거**

Arduino UNO Q는 Qualcomm QRB2210(4× Cortex-A53, Linux)과 STM32U585(Cortex-M33, Zephyr RTOS)가 하나의 보드에 통합된 이중 프로세서 구조를 갖는다. 이 구조는 단순한 UNO 대비 아래와 같은 차별화된 기능을 제공한다.

| 항목 | 일반 Arduino UNO | Arduino UNO Q |
|---|---|---|
| 프로세서 | ATmega328P (8-bit, 16MHz) | QRB2210 (64-bit, 2GHz) + STM32U585 |
| OS 지원 | 불가 | Linux (Ubuntu) 내장 |
| ROS 2 직접 실행 | 불가 (PC 연결 필수) | 보드 자체에서 실행 가능 |
| 실시간 제어 | MCU 단독 | STM32 담당 (μs 정밀도) |
| App 개발 | 불가 | App Lab (HTML/JS 대시보드) |

본 프로젝트에서 ROS 2를 사용하는 이유는 센서 데이터 처리(imu_filter_node), 제어 로직(obstacle_avoider), 시뮬레이션(turtlesim)을 독립적인 노드로 분리하고 토픽 기반으로 연결함으로써 모듈화·재사용성·확장성을 확보하기 위함이다. 이 구조는 UNO Q의 Linux 프로세서 없이는 구현할 수 없다.

---

## 4. 필요기술

### 하드웨어
| 부품 | 역할 |
|---|---|
| Arduino UNO Q | 이중 프로세서 메인 보드 |
| MPU-9250 | 9축 IMU (가속도 ±4g / 자이로 ±500dps / 지자기) |
| HC-SR04 | 초음파 거리 센서 (2cm ~ 4m) |
| SG90 서보모터 | HC-SR04 방향 제어 (0° ~ 180°) |

### 소프트웨어 / 프레임워크
| 기술 | 적용 내용 |
|---|---|
| ROS 2 Jazzy | 노드 간 토픽 통신, turtlesim 시뮬레이션 |
| Arduino (Zephyr) | STM32 펌웨어, I2C/Serial/Servo 제어 |
| 상보 필터 (Complementary Filter) | Roll/Pitch: α=0.98, Yaw: α=0.90 (tilt-compensated 자기 편각) |
| rosbridge + roslibjs | WebSocket 기반 브라우저↔ROS 2 실시간 연동 |
| Python (rclpy) | Serial→ROS 2 브리지 노드 |
| C++ (rclcpp) | 고성능 필터·제어 노드 |

### 핵심 알고리즘
- **상보 필터**: 가속도계(저주파 안정) + 자이로(고주파 정확) 융합
- **Tilt-compensated 자기 편각**: Roll/Pitch 보정 후 Yaw 계산
- **비례 장애물 회피**: 거리에 비례한 감속 및 회피 방향 편향

---

## 개발 환경

- **OS**: Ubuntu 24.04 (WSL2 on Windows 11)
- **ROS 2**: Jazzy Jalisco
- **Arduino IDE**: Zephyr 1.0.0 (arduino:zephyr)
- **패키지**: `imu_turtle_demo`
