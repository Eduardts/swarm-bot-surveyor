#!/usr/bin/env python
"""
Exporter tool: subscribes to /merged_cloud and writes it to disk as a simple XYZ or PLY file.
If open3d is available, it writes PLY; otherwise it writes a text XYZ file.
Usage: rosrun swarm-bot-surveyor exporter.py --output /tmp/survey.ply
"""

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import argparse
import time
import os

try:
    import open3d as o3d
    HAVE_O3D = True
except Exception:
    HAVE_O3D = False

class Exporter(object):
    def __init__(self, out_path):
        rospy.init_node('exporter', anonymous=True)
        self.out_path = out_path
        self.received = False
        rospy.Subscriber('merged_cloud', PointCloud2, self.cb)
        rospy.loginfo('exporter waiting for merged_cloud; will write to %s', out_path)
        # wait for data or timeout
        timeout = rospy.get_param('~timeout', 30)
        start = time.time()
        while not rospy.is_shutdown() and (time.time() - start) < timeout and not self.received:
            rospy.sleep(0.1)
        if not self.received:
            rospy.logwarn('No merged_cloud received before timeout')
        else:
            rospy.loginfo('Export complete: %s', out_path)

    def cb(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        if not points:
            rospy.logwarn('Received empty cloud')
            return
        # write output
        base, ext = os.path.splitext(self.out_path)
        if HAVE_O3D and ext.lower() in ('.ply', '.pcd'):
            pc = o3d.geometry.PointCloud()
            import numpy as np
            pc.points = o3d.utility.Vector3dVector(np.array(points))
            o3d.io.write_point_cloud(self.out_path, pc)
            rospy.loginfo('Wrote PLY/PCD using open3d')
        else:
            # fallback: write simple XYZ
            with open(self.out_path, 'w') as f:
                for p in points:
                    f.write('%f %f %f\n' % (p[0], p[1], p[2]))
            rospy.loginfo('Wrote simple XYZ to %s', self.out_path)
        self.received = True

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o', default='/tmp/survey.xyz', help='Output filename (ply/pcd/xyz)')
    args, unknown = parser.parse_known_args()
    try:
        Exporter(args.output)
    except rospy.ROSInterruptException:
        pass
