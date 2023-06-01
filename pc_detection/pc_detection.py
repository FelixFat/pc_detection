import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Int32, Float32, Float32MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import numpy as np
from sklearn.cluster import KMeans


class PCDetection(Node):

    def __init__(self):
        super().__init__('pc_detection')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.listener_callback,
            10)
        
        self.pub_cluster = self.create_publisher(PointCloud2, '/pc_detection/cluster', 10)
        self.pub_centroid = self.create_publisher(Float32MultiArray, '/pc_detection/centroid', 10)
        self.pub_distance = self.create_publisher(Float32, '/pc_detection/distance', 10)
        self.pub_points_num = self.create_publisher(Int32, '/pc_detection/points_num', 10)
        
        timer_period = 2.0  # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cloud = np.array([])

    def listener_callback(self, msg):
        self.cloud = np. array([
            np.array([data[0], data[1], data[2]])
            for data in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ])

    def timer_callback(self):

        msg_cloud = PointCloud2()
        header = Header()
        header.frame_id = 'camera_link'

        msg_centroid = Float32MultiArray()
        msg_distance = Float32()
        msg_points_num = Int32()

        if len(self.cloud) > 10:
            kmeans = KMeans(
                n_clusters=2,
                init='random',
                n_init=10,
                max_iter=100,
                random_state=42,
                algorithm='lloyd'
                ).fit(self.cloud)

            clusters = [
                np.array([point for i, point in enumerate(self.cloud) if kmeans.labels_[i] == l])
                for l in set(kmeans.labels_) if l != -1
            ]
            
            distances = [
                np.median(clusters[i][:, 2])
                for i in range(len(clusters))
            ]

            dist = np.min(distances)
            cluster = clusters[distances.index(dist)]
            centroid = kmeans.cluster_centers_[distances.index(dist)]

            msg_cloud = pc2.create_cloud_xyz32(header, cluster)
            msg_centroid.data = list(centroid)
            msg_distance.data = dist
            msg_points_num.data = len(cluster)
            
            self.get_logger().info(f'Median distance is {dist}, '
                                   f'number of points in {len(cluster)}, '
                                   f'centroid is {centroid}')

        else:
            msg_cloud = pc2.create_cloud_xyz32(header, [])
            msg_centroid.data = [0.0, 0.0, 0.0]
            msg_distance.data = 0.0
            msg_points_num.data = 0
            self.get_logger().info(f'WARNING: No data!')

        self.pub_cluster.publish(msg_cloud)
        self.pub_centroid.publish(msg_centroid)
        self.pub_distance.publish(msg_distance)
        self.pub_points_num.publish(msg_points_num)


def main(args=None):
    rclpy.init(args=args)

    pc_detection = PCDetection()
    rclpy.spin(pc_detection)
    
    pc_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

