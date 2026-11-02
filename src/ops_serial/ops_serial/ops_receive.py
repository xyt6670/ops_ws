import math
import struct

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial


FRAME_HEADER = 0x5C
FRAME_LENGTH = 14


def crc8(data: bytes) -> int:
    """与 OPS 下位机 DJI CRC8 算法保持一致。"""
    crc = 0xFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1

    return crc & 0xFF


class OpsReceiver(Node):

    def __init__(self):
        super().__init__('ops_receiver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0
        )

        self.rx_buffer = bytearray()
        self.frame_count = 0
        self.crc_error_count = 0

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/ops/odom_raw',
            10
        )

        # 每5ms检查一次串口，不代表必须每5ms收到新数据
        self.timer = self.create_timer(0.005, self.read_serial)

        self.get_logger().info(
            f'OPS串口已打开：{port}，波特率：{baudrate}'
        )

    def read_serial(self):
        waiting = self.serial_port.in_waiting

        if waiting > 0:
            self.rx_buffer.extend(
                self.serial_port.read(waiting)
            )

        self.parse_buffer()

    def parse_buffer(self):
        while True:
            header_index = self.rx_buffer.find(
                bytes([FRAME_HEADER])
            )

            if header_index < 0:
                self.rx_buffer.clear()
                return

            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < FRAME_LENGTH:
                return

            frame = bytes(self.rx_buffer[:FRAME_LENGTH])

            expected_crc = crc8(frame[:13])
            received_crc = frame[13]

            if expected_crc != received_crc:
                self.crc_error_count += 1

                # CRC错误时只扔掉一个字节，再重新寻找帧头
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:FRAME_LENGTH]

            x, y, yaw = struct.unpack(
                '<fff',
                frame[1:13]
            )

            self.publish_odometry(x, y, yaw)

    def publish_odometry(self, x, y, yaw):
        message = Odometry()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'

        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.position.z = 0.0
        

        # 把平面yaw角转换为四元数
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 第一阶段暂时不填写速度
        message.twist.twist.linear.x = 0.0
        message.twist.twist.linear.y = 0.0
        message.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(message)

        self.frame_count += 1

        # 每50帧打印一次，避免疯狂刷屏
        if self.frame_count % 50 == 0:
            self.get_logger().info(
                f'x={x:.4f} m, '
                f'y={y:.4f} m, '
                f'yaw={math.degrees(yaw):.2f} deg, '
                f'crc_errors={self.crc_error_count}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = OpsReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
