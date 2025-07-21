import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # 添加 HistoryPolicy
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import OccupancyGrid
from autoware_perception_msgs.msg import TrafficLightGroupArray  # 修改为 TrafficLightGroupArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header  # 添加导入 Header
import struct  # 添加 struct 用于打包点云数据
from autoware_vehicle_msgs.msg import ControlModeReport  # 新增导入 ControlModeReport

class MultiTopicPublisher(Node):
    def __init__(self):
        super().__init__('multi_topic_publisher')

        # 创建 reliable QoS 配置（用于其他话题）
        qos_profile_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # 创建 best effort QoS 配置（用于 pointcloud，匹配 Autoware 传感器数据）
        qos_profile_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5  # 常见传感器 QoS depth
        )

        # 创建发布器，使用相应 QoS 配置
        self.predicted_objects_publisher = self.create_publisher(
            PredictedObjects, '/perception/object_recognition/objects', qos_profile_reliable
        )
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid, '/perception/occupancy_grid_map/map', qos_profile_reliable
        )
        self.traffic_signals_publisher = self.create_publisher(
            TrafficLightGroupArray, '/perception/traffic_light_recognition/traffic_signals', qos_profile_reliable  # 修改为 TrafficLightGroupArray
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/perception/obstacle_segmentation/pointcloud', qos_profile_best  # 修改为 best effort QoS
        )
        self.control_mode_publisher = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', qos_profile_reliable  # 新增控制模式发布器
        )

        # 定时器分别发布五个消息
        self.timer = self.create_timer(0.1, self.publish_messages)
        #self.get_logger().info("MultiTopicPublisher node has started with reliable QoS.")

    def publish_messages(self):
        self.publish_predicted_objects()
        self.publish_occupancy_grid()
        self.publish_traffic_signals()
        self.publish_pointcloud()
        self.publish_control_mode()  # 新增发布方法

    def publish_predicted_objects(self):
        predicted_objects = PredictedObjects()
        predicted_objects.header.stamp = self.get_clock().now().to_msg()
        predicted_objects.header.frame_id = "base_link"  # 修改为 base_link
        predicted_objects.objects = []  # 确保 objects 字段为空

        self.predicted_objects_publisher.publish(predicted_objects)
        #self.get_logger().info("Published empty PredictedObjects message.")

    def publish_occupancy_grid(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"  # 保持为 map

        occupancy_grid.info.map_load_time.sec = 0
        occupancy_grid.info.map_load_time.nanosec = 0
        occupancy_grid.info.resolution = 0.5
        occupancy_grid.info.width = 300
        occupancy_grid.info.height = 300
        occupancy_grid.info.origin.position.x = -74.5
        occupancy_grid.info.origin.position.y = -72.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        occupancy_grid.data = [1] * (occupancy_grid.info.width * occupancy_grid.info.height)

        self.occupancy_grid_publisher.publish(occupancy_grid)
        #self.get_logger().info("Published OccupancyGrid message with current timestamp.")

    def publish_traffic_signals(self):
        traffic_signals = TrafficLightGroupArray()
        traffic_signals.traffic_light_groups = []  # 修正为 traffic_light_groups 字段，并设置为空

        self.traffic_signals_publisher.publish(traffic_signals)
        #self.get_logger().info("Published empty TrafficLightGroupArray message.")

    def publish_pointcloud(self):
        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = "base_link"  # 修改为 base_link

        pointcloud.height = 1
        pointcloud.width = 1  # 修改为 1，为发布一个 dummy 点
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
            PointField(name='intensity', offset=12, datatype=7, count=1)  # 添加 intensity 字段
        ]
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16  # 4 * 4 = 16
        pointcloud.row_step = pointcloud.point_step * pointcloud.width  # 更新 row_step
        # 添加 dummy 数据
        pointcloud.data = struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)
        pointcloud.is_dense = True

        self.pointcloud_publisher.publish(pointcloud)
        #self.get_logger().info("Published PointCloud2 message with dummy point.")

    def publish_control_mode(self):
        control_mode = ControlModeReport()
        control_mode.stamp = self.get_clock().now().to_msg()  # 修改为 stamp（无 header）
        control_mode.mode = 1  # 默认设置为 MANUAL 模式（根据最新消息定义，4 表示 MANUAL）

        self.control_mode_publisher.publish(control_mode)
        #self.get_logger().info("Published ControlModeReport message with mode=4.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()