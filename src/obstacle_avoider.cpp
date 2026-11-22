#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// 메인 제어 노드
//
// 입력:
//   /imu/filtered    - 상보 필터 처리된 IMU (사용자 의도: 기울기 → 이동 방향)
//   /ultrasonic/range - 초음파 거리 (장애물까지 거리)
//   /turtle1/pose    - turtle 현재 방향 (서보 각도 계산용)
//
// 출력:
//   /turtle1/cmd_vel - turtle 속도 명령 (회피 보정 포함)
//   /servo/angle     - 서보 각도 (turtle 진행 방향으로 초음파 조준)
//   /led/cmd         - LED 매트릭스 + 버저 명령

class ObstacleAvoider : public rclcpp::Node {
public:
  ObstacleAvoider() : Node("obstacle_avoider") {
    declare_parameter("linear_gain",  2.0);
    declare_parameter("angular_gain", 2.0);
    declare_parameter("danger_dist",  0.30);  // m: 즉시 회피
    declare_parameter("warn_dist",    0.60);  // m: 감속 시작

    linear_gain_  = get_parameter("linear_gain").as_double();
    angular_gain_ = get_parameter("angular_gain").as_double();
    danger_dist_  = get_parameter("danger_dist").as_double();
    warn_dist_    = get_parameter("warn_dist").as_double();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/filtered", 10,
      std::bind(&ObstacleAvoider::imuCb, this, std::placeholders::_1));
    range_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/ultrasonic/range", 10,
      std::bind(&ObstacleAvoider::rangeCb, this, std::placeholders::_1));
    pose_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&ObstacleAvoider::poseCb, this, std::placeholders::_1));

    cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>("/servo/angle", 10);
    led_pub_   = create_publisher<std_msgs::msg::String>("/led/cmd", 10);

    RCLCPP_INFO(get_logger(), "Obstacle avoider started (danger=%.2fm, warn=%.2fm)",
                danger_dist_, warn_dist_);
  }

private:
  double linear_gain_, angular_gain_, danger_dist_, warn_dist_;
  double range_ = 999.0;
  double turtle_theta_ = 0.0;

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 필터링된 쿼터니언에서 roll/pitch 추출
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // IMU 기울기 → 사용자가 의도한 turtle 속도
    double user_linear  = std::clamp(-pitch * linear_gain_,  -2.0, 2.0);
    double user_angular = std::clamp(-roll  * angular_gain_, -2.0, 2.0);

    // 서보를 turtle 진행 방향으로 조준
    // turtle_theta: [-π, π] → 서보: [0°, 180°]
    float servo_deg = static_cast<float>(
      std::clamp((turtle_theta_ / M_PI + 1.0) * 90.0, 0.0, 180.0));
    auto servo_msg = std_msgs::msg::Float32();
    servo_msg.data = servo_deg;
    servo_pub_->publish(servo_msg);

    // 장애물 회피 로직
    geometry_msgs::msg::Twist cmd;
    std::string led_state;

    if (range_ < danger_dist_) {
      // 위험: 후진 + 회전 이탈
      cmd.linear.x  = -0.5;
      cmd.angular.z = (user_angular >= 0.0) ? 2.0 : -2.0;
      led_state = "DANGER";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "DANGER: %.2fm", range_);

    } else if (range_ < warn_dist_) {
      // 경고: 속도 비례 감소 + 회피 방향 편향
      double scale = (range_ - danger_dist_) / (warn_dist_ - danger_dist_);
      double bias  = (1.0 - scale) * 1.5;
      cmd.linear.x  = user_linear * scale;
      cmd.angular.z = std::clamp(
        user_angular + ((user_angular >= 0.0) ? bias : -bias), -2.0, 2.0);
      led_state = "WARNING";

    } else {
      // 안전: 사용자 의도 그대로 전달
      cmd.linear.x  = user_linear;
      cmd.angular.z = user_angular;

      if (std::abs(pitch) < 0.1 && std::abs(roll) < 0.1)
        led_state = "IDLE";
      else if (std::abs(pitch) >= std::abs(roll))
        led_state = (pitch < 0.0) ? "FORWARD" : "BACKWARD";
      else
        led_state = (roll  < 0.0) ? "LEFT" : "RIGHT";
    }

    cmd_pub_->publish(cmd);
    auto led_msg = std_msgs::msg::String();
    led_msg.data = led_state;
    led_pub_->publish(led_msg);
  }

  void rangeCb(const sensor_msgs::msg::Range::SharedPtr msg) {
    range_ = msg->range;
  }

  void poseCb(const turtlesim::msg::Pose::SharedPtr msg) {
    turtle_theta_ = msg->theta;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}
