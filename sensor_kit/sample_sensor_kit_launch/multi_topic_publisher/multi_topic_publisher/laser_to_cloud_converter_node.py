import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
import argparse
import array

class LaserToCloudConverter(Node):
    def __init__(self, input_topic1, output_topic1, input_topic2, output_topic2):
        super().__init__('laser_to_cloud_converter')
        # 定义兼容的 QoS profile: BEST_EFFORT 以匹配发布者
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # 深度可根据需要调整，默认为10
        )
        # 创建订阅者和发布者，使用兼容 QoS
        self.sub1 = self.create_subscription(LaserScan, input_topic1, self.callback1, qos_profile=qos_profile)
        self.pub1 = self.create_publisher(PointCloud2, output_topic1, 10)
        self.sub2 = self.create_subscription(LaserScan, input_topic2, self.callback2, qos_profile=qos_profile)
        self.pub2 = self.create_publisher(PointCloud2, output_topic2, 10)
        self.get_logger().info(f'Subscribing to {input_topic1} and {input_topic2}, publishing to {output_topic1} and {output_topic2}')

    def callback1(self, msg: LaserScan):
        cloud = self.convert_laser_to_cloud(msg)
        self.pub1.publish(cloud)

    def callback2(self, msg: LaserScan):
        cloud = self.convert_laser_to_cloud(msg)
        self.pub2.publish(cloud)

    def convert_laser_to_cloud(self, scan: LaserScan) -> PointCloud2:
        # 准备 PointCloud2 消息
        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = scan.header.stamp
        cloud.header.frame_id = scan.header.frame_id
        # 定义字段: 根据参考 XYZI (intensity UINT8) return_type UINT8 channel UINT16 azimuth FLOAT32 elevation FLOAT32 distance FLOAT32 time UINT32
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
            PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=28, datatype=PointField.UINT32, count=1)
        ]
        # 每个点的字节大小: 32 bytes (fff B B H f f f I)
        point_step = 32
        cloud.point_step = point_step
        cloud.height = 1
        cloud.is_bigendian = False
        cloud.is_dense = False  # 包含无效点
        # 构建数据，包含所有点（无效点设置 NaN/0）
        data = []
        num_points = len(scan.ranges)
        time_increment = scan.time_increment if scan.time_increment else 0.0
        for i in range(num_points):
            r = scan.ranges[i]
            invalid = math.isinf(r) or math.isnan(r) or r < scan.range_min or r > scan.range_max
            if invalid:
                x = math.nan
                y = math.nan
                z = math.nan
                intensity = 0
                return_type = 0
                channel = 0
                azimuth = 0.0
                elevation = 0.0
                distance = 0.0
                time_ns = 0
            else:
                angle = scan.angle_min + i * scan.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # 假设2D激光
                intensity_float = scan.intensities[i] if i < len(scan.intensities) else 0.0
                intensity = max(0, min(255, int(intensity_float)))  # clamp to UINT8
                return_type = 0  # 假设单返回激光
                channel = 0  # 假设单通道
                azimuth = math.atan2(y, x)  # A: atan2(Y, X)
                elevation = 0.0  # 假设2D激光
                distance = math.hypot(x, y, z)  # D: hypot(X, Y, Z)
                time_ns = int(i * time_increment * 1e9) & 0xffffffff  # 时间偏移 (UINT32, ns)
            # 打包成 binary
            point_data = struct.pack('fffBBHfffI', x, y, z, intensity, return_type, channel, azimuth, elevation, distance, time_ns)
            data.extend(array.array('B', point_data))  # 转为字节数组
        cloud.data = data
        cloud.width = num_points
        cloud.row_step = point_step * num_points
        return cloud

def main(args=None):

    rclpy.init(args=None)
    node = LaserToCloudConverter('/scan_f', '/sensing/lidar/front/pointcloud_raw_ex', '/scan_b', '/sensing/lidar/back/pointcloud_raw_ex')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
