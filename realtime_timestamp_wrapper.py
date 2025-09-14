#!/usr/bin/env python3
"""
Real-Time Timestamp Wrapper
Suscribe a topics del dataset y republica con timestamps actuales
Simula sensores reales para testing de sincronización
"""

import rospy
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix, CompressedImage
import copy

class RealTimeWrapper:
    def __init__(self):
        rospy.init_node('realtime_timestamp_wrapper')

        # Subscribers to dataset topics
        self.lidar_sub = rospy.Subscriber('/rslidar_points', PointCloud2, self.lidar_callback)
        self.imu_sub = rospy.Subscriber('/imu_raw', Imu, self.imu_callback)
        self.gnss_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.gnss_callback)

        # Multiple camera subscribers (cam0-cam5)
        self.camera_subs = []
        self.camera_pubs = []
        for cam_id in range(6):
            sub = rospy.Subscriber(f'/camera_array/cam{cam_id}/image_raw/compressed',
                                 CompressedImage,
                                 lambda msg, id=cam_id: self.camera_callback(msg, id))
            pub = rospy.Publisher(f'/sensors/camera{cam_id}_realtime', CompressedImage, queue_size=10)
            self.camera_subs.append(sub)
            self.camera_pubs.append(pub)

        # Publishers with real-time timestamps
        self.lidar_pub = rospy.Publisher('/sensors/lidar_realtime', PointCloud2, queue_size=10)
        self.imu_pub = rospy.Publisher('/sensors/imu_realtime', Imu, queue_size=10)
        self.gnss_pub = rospy.Publisher('/sensors/gnss_realtime', NavSatFix, queue_size=10)
        
        rospy.loginfo("Real-Time Timestamp Wrapper initialized")
        rospy.loginfo("Converting dataset timestamps to real-time for sensor fusion testing")
        rospy.loginfo(f"Subscribed to 6 camera topics: cam0-cam5")
        
    def lidar_callback(self, msg):
        rospy.logdebug("LiDAR callback received")
        real_msg = copy.deepcopy(msg)
        real_msg.header.stamp = rospy.Time.now()
        self.lidar_pub.publish(real_msg)

    def imu_callback(self, msg):
        rospy.logdebug("IMU callback received")
        real_msg = copy.deepcopy(msg)
        real_msg.header.stamp = rospy.Time.now()
        self.imu_pub.publish(real_msg)

    def gnss_callback(self, msg):
        rospy.logdebug("GNSS callback received")
        real_msg = copy.deepcopy(msg)
        real_msg.header.stamp = rospy.Time.now()
        self.gnss_pub.publish(real_msg)

    def camera_callback(self, msg, camera_id):
        rospy.logdebug(f"Camera {camera_id} callback received")
        real_msg = copy.deepcopy(msg)
        real_msg.header.stamp = rospy.Time.now()
        self.camera_pubs[camera_id].publish(real_msg)

if __name__ == '__main__':
    try:
        wrapper = RealTimeWrapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass