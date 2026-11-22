#!/usr/bin/env python3
"""
Serial → ROS 2 IMU 브리지
STM32가 보내는 CSV(ax,ay,az,gx,gy,gz,mx,my,mz)를 읽어
/imu/raw, /imu/mag 토픽으로 발행

사용법:
  ros2 run imu_turtle_demo serial_imu_bridge --ros-args -p port:=/dev/ttyACM0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import threading


class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.imu_pub = self.create_publisher(Imu, '/imu/raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            raise

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 9:
                    continue
                ax, ay, az, gx, gy, gz, mx, my, mz = [float(p) for p in parts]
            except (ValueError, serial.SerialException):
                continue

            now = self.get_clock().now().to_msg()

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            imu.orientation_covariance[0] = -1.0  # orientation 미계산
            self.imu_pub.publish(imu)

            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = 'imu_link'
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = my
            mag.magnetic_field.z = mz
            self.mag_pub.publish(mag)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
