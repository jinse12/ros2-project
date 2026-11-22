#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// 상보 필터 (Complementary Filter)
// - Accel: 절대 각도 계산 가능하지만 진동에 노이즈
// - Gyro: 순간 회전 정확하지만 시간이 지나면 drift
// - Mag: yaw drift 보정 (지자기 절대 방위)
// 결과: roll/pitch/yaw가 담긴 orientation 쿼터니언을 /imu/filtered로 발행

class ImuFilterNode : public rclcpp::Node {
public:
  ImuFilterNode()
  : Node("imu_filter_node"),
    roll_(0.0), pitch_(0.0), yaw_(0.0),
    mag_yaw_(0.0), initialized_(false), has_mag_(false)
  {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/raw", 10,
      std::bind(&ImuFilterNode::imuCb, this, std::placeholders::_1));
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "/imu/mag", 10,
      std::bind(&ImuFilterNode::magCb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
    RCLCPP_INFO(get_logger(), "IMU filter node started");
  }

private:
  static constexpr double ALPHA     = 0.98;  // accel: 2%, gyro: 98%
  static constexpr double ALPHA_YAW = 0.90;  // mag: 10%, gyro: 90%

  double roll_, pitch_, yaw_, mag_yaw_;
  bool initialized_, has_mag_;
  rclcpp::Time last_time_;

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    // 가속도계로 계산한 절대 각도 (노이즈 있지만 drift 없음)
    double accel_roll  = std::atan2(ay, std::sqrt(ax*ax + az*az));
    double accel_pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    if (!initialized_) {
      roll_ = accel_roll;
      pitch_ = accel_pitch;
      last_time_ = msg->header.stamp;
      initialized_ = true;
      return;
    }

    double dt = (rclcpp::Time(msg->header.stamp) - last_time_).seconds();
    last_time_ = msg->header.stamp;
    if (dt <= 0.0 || dt > 0.5) return;

    // 상보 필터: gyro 적분 + accel 보정
    roll_  = ALPHA * (roll_  + gx * dt) + (1.0 - ALPHA) * accel_roll;
    pitch_ = ALPHA * (pitch_ + gy * dt) + (1.0 - ALPHA) * accel_pitch;

    // yaw: mag 있으면 보정, 없으면 gyro만 적분
    if (has_mag_) {
      yaw_ = ALPHA_YAW * (yaw_ + gz * dt) + (1.0 - ALPHA_YAW) * mag_yaw_;
    } else {
      yaw_ += gz * dt;
    }

    // 필터링된 orientation을 쿼터니언으로 변환하여 발행
    auto out = sensor_msgs::msg::Imu(*msg);
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    out.orientation.w = q.w();
    out.orientation_covariance[0] = 0.001;  // 유효한 orientation 표시
    pub_->publish(out);
  }

  // 기울기 보정된 지자기 yaw 계산
  void magCb(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    double mx = msg->magnetic_field.x;
    double my = msg->magnetic_field.y;
    double mz = msg->magnetic_field.z;

    double mx_c = mx*std::cos(pitch_) + mz*std::sin(pitch_);
    double my_c = mx*std::sin(roll_)*std::sin(pitch_)
                + my*std::cos(roll_)
                - mz*std::sin(roll_)*std::cos(pitch_);
    mag_yaw_ = std::atan2(-my_c, mx_c);
    has_mag_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFilterNode>());
  rclcpp::shutdown();
  return 0;
}
