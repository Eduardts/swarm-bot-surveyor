#!/usr/bin/env python
"""
A lightweight SLAM processor: subscribes to multiple drone point clouds, aggregates them, and republishes
as /merged_cloud. This is a placeholder for a true SLAM pipeline (e.g., RTAB-Map, Cartographer, ORB-SLAM).
"""

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import threading

class SlamProcessor(object):
    def __init__(self):
        rospy.init_node('slam_processor', anonymous=True)
        self.drone_count = rospy.get_param('~num_drones', rospy.get_param('/num_drones', 3))
        # subscribe pattern: drone_{i}/cloud
        self.subs = []
        self.buffer = []
        self.lock = threading.Lock()
        for i in range(self.drone_count):
            topic = 'drone_{}/cloud'.format(i)
            sub = rospy.Subscriber(topic, PointCloud2, self.cloud_cb, callback_args=i)
            self.subs.append(sub)
        self.pub = rospy.Publisher('merged_cloud', PointCloud2, queue_size=1)
        self.publish_rate = rospy.Rate(1)  # 1 Hz aggregation
        rospy.loginfo('slam_processor started for %d drones', self.drone_count)
        self.run()

    def cloud_cb(self, msg, drone_id):
        # store last cloud from a drone
        with self.lock:
            # store as (drone_id, msg)
            found = False
            for idx, (d, _) in enumerate(self.buffer):
                if d == drone_id:
                    self.buffer[idx] = (drone_id, msg)
                    found = True
                    break
            if not found:
                self.buffer.append((drone_id, msg))

    def run(self):
        seq = 0
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            with self.lock:
                if not self.buffer:
                    continue
                # merge all points in buffer
                merged_points = []
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'map'
                for (d, cloud_msg) in list(self.buffer):
                    for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                        merged_points.append((p[0], p[1], p[2]))
                if not merged_points:
                    continue
                merged_msg = pc2.create_cloud_xyz32(header, merged_points)
                merged_msg.header.seq = seq
                self.pub.publish(merged_msg)
                seq += 1

if __name__ == '__main__':
    try:
        SlamProcessor()
    except rospy.ROSInterruptException:
        pass
