#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <cmath>
#include <cstdlib>

// 하드웨어 없이 테스트할 때 사용하는 IMU/초음파 시뮬레이터
// - IMU: 천천히 기울었다 세웠다 하는 동작 + 노이즈
// - 초음파: 가까웠다 멀어졌다 반복 (장애물 감지 테스트)

class SimImuPublisher : public rclcpp::Node {
public:
  SimImuPublisher() : Node("sim_imu_publisher"), t_(0.0) {
    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
    mag_pub_   = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    range_pub_ = create_publisher<sensor_msgs::msg::Range>("/ultrasonic/range", 10);

    imu_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),   // 100 Hz
      std::bind(&SimImuPublisher::pubImu, this));
    range_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&SimImuPublisher::pubRange, this));

    RCLCPP_INFO(get_logger(), "Simulator started — no hardware required");
  }

private:
  double t_;

  double noise(double scale) {
    return scale * (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
  }

  void pubImu() {
    t_ += 0.01;
    const double g = 9.81;

    // 완만한 기울기 변화 (실제 사람이 보드를 기울이는 느낌)
    double pitch = 0.25 * std::sin(0.4 * t_);
    double roll  = 0.15 * std::sin(0.25 * t_ + 1.0);

    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp    = now();
    imu.header.frame_id = "imu";

    // 가속도: 중력 + 노이즈 (진동 시뮬레이션)
    imu.linear_acceleration.x = -g * std::sin(pitch) + noise(0.3);
    imu.linear_acceleration.y =  g * std::sin(roll)  + noise(0.3);
    imu.linear_acceleration.z =  g * std::cos(pitch) * std::cos(roll) + noise(0.1);

    // 자이로: 각속도 + 바이어스 drift (필터 필요성 시연)
    imu.angular_velocity.x = 0.0  + noise(0.02);
    imu.angular_velocity.y = 0.0  + noise(0.02);
    imu.angular_velocity.z = 0.02 + noise(0.015);  // 작은 drift bias

    imu.orientation_covariance[0] = -1.0;  // orientation 미계산 표시
    imu_pub_->publish(imu);

    // 지자기: 북쪽 기준 천천히 회전
    double yaw = 0.08 * t_;
    auto mag = sensor_msgs::msg::MagneticField();
    mag.header = imu.header;
    const double m = 50e-6;
    mag.magnetic_field.x =  m * std::cos(yaw) + noise(1e-6);
    mag.magnetic_field.y = -m * std::sin(yaw) + noise(1e-6);
    mag.magnetic_field.z = 20e-6;
    mag_pub_->publish(mag);
  }

  void pubRange() {
    // 장애물이 0.2m ~ 1.5m 사이를 오가며 접근/이탈
    double r = 0.85 + 0.65 * std::sin(0.15 * t_);

    auto range = sensor_msgs::msg::Range();
    range.header.stamp    = now();
    range.header.frame_id = "ultrasonic";
    range.radiation_type  = sensor_msgs::msg::Range::ULTRASOUND;
    range.field_of_view   = 0.2618f;  // 15도
    range.min_range       = 0.02f;
    range.max_range       = 4.0f;
    range.range           = static_cast<float>(r);
    range_pub_->publish(range);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::TimerBase::SharedPtr imu_timer_, range_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
