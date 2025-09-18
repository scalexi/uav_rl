import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from px4_msgs.msg import ObstacleDistance
import numpy as np
import math

class ObstacleDistanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_distance')
        self.subscription = self.create_subscription(PointCloud2, '/depth_camera/points', self.callback, 10)
        self.publisher = self.create_publisher(ObstacleDistance, '/fmu/in/obstacle_distance', 10)
        self.get_logger().info('Publishing sectoral obstacle distances to PX4...')

    def callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if points.size == 0:
            return

        num_sectors = 72
        increment_deg = 360 / num_sectors
        min_dist_cm = 10
        max_dist_cm = 5000
        invalid_dist = 65535
        distances = np.full(num_sectors, invalid_dist, dtype=np.uint16)

        valid_points = points[(points[:, 2] > 0.1) & (np.abs(points[:, 1]) < 2.0)]

        for point in valid_points:
            x, y, z = point
            theta = math.degrees(math.atan2(x, z))
            sector = int((theta + 180) / increment_deg) % num_sectors
            dist_cm = int(z * 100)
            if min_dist_cm < dist_cm < max_dist_cm:
                distances[sector] = min(distances[sector], dist_cm)

        obs_msg = ObstacleDistance()
        obs_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        obs_msg.frame = 8  # MAV_FRAME_BODY_FRD
        obs_msg.min_distance = min_dist_cm
        obs_msg.max_distance = max_dist_cm
        obs_msg.angle_offset = 0.0
        obs_msg.increment = increment_deg
        obs_msg.distances = distances.tolist()
        self.publisher.publish(obs_msg)
        self.get_logger().info(f'Published distances (nearest: {np.min(distances[distances < invalid_dist]):.0f} cm)')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
