import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import OccupancyGrid
from autoware_perception_msgs.msg import TrafficSignalArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header  # 添加导入 Header

class MultiTopicPublisher(Node):
    def __init__(self):
        super().__init__('multi_topic_publisher')

        # 创建 QoS 配置（设置为 reliable）
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # 创建发布器，使用 QoS 配置
        self.predicted_objects_publisher = self.create_publisher(
            PredictedObjects, '/perception/object_recognition/objects', qos_profile
        )
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid, '/perception/occupancy_grid_map/map', qos_profile
        )
        self.traffic_signals_publisher = self.create_publisher(
            TrafficSignalArray, '/perception/traffic_light_recognition/traffic_signals', qos_profile
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/perception/obstacle_segmentation/pointcloud', qos_profile
        )

        # 定时器分别发布四个消息
        self.timer = self.create_timer(0.1, self.publish_messages)
        #self.get_logger().info("MultiTopicPublisher node has started with reliable QoS.")

    def publish_messages(self):
        self.publish_predicted_objects()
        self.publish_occupancy_grid()
        self.publish_traffic_signals()
        self.publish_pointcloud()

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
        traffic_signals = TrafficSignalArray()
        traffic_signals.signals = []

        self.traffic_signals_publisher.publish(traffic_signals)
        #self.get_logger().info("Published empty TrafficSignalArray message.")

    def publish_pointcloud(self):
        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = "base_link"  # 修改为 base_link

        pointcloud.height = 1
        pointcloud.width = 0
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1)
        ]
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        pointcloud.row_step = 0
        pointcloud.data = []
        pointcloud.is_dense = True

        self.pointcloud_publisher.publish(pointcloud)
        #self.get_logger().info("Published empty PointCloud2 message with current timestamp.")

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