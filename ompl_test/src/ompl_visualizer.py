#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from ompl_test.msg import *
np.set_printoptions(precision=4)


class PathVisualizer(object):

    def __init__(self):
        rospy.init_node('visualizeNode', anonymous=True)
        self.init_angles = []
        self.init_pos = []
        # initialize a yumi robot
        # the list of poses
        self.traj_pts = []
        self.traj_pts_update = []
        # the list of joint states from dmp
        self.traj = []
        # the failed points
        self.fail_pts = set()

    def visualize_result(self):
        markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)
        traj_states = []

        pt_marker = Marker()
        pt_marker.header.frame_id = "World"
        pt_marker.header.stamp = rospy.get_rostime()
        pt_marker.ns = "World"
        pt_marker.id = 0
        pt_marker.type = 5
        pt_marker.action = 0
        pt_marker.scale.x = 0.02
        pt_marker.scale.y = 0.02
        pt_marker.scale.z = 0.02
        pt_marker.color.r = 0.0
        pt_marker.color.g = 1.0
        pt_marker.color.b = 0.0
        pt_marker.color.a = 1.0

        pt_start_marker = Marker()
        pt_start_marker.header.frame_id = "World"
        pt_start_marker.header.stamp = rospy.get_rostime()
        pt_start_marker.ns = "World"
        pt_start_marker.id = 0
        pt_start_marker.type = 7
        pt_start_marker.action = 0
        pt_start_marker.scale.x = 0.02
        pt_start_marker.scale.y = 0.02
        pt_start_marker.scale.z = 0.02
        pt_start_marker.color.r = 0.0
        pt_start_marker.color.g = 0.0
        pt_start_marker.color.b = 1.0
        pt_start_marker.color.a = 1.0

        pt_end_marker = Marker()
        pt_end_marker.header.frame_id = "World"
        pt_end_marker.header.stamp = rospy.get_rostime()
        pt_end_marker.ns = "World"
        pt_end_marker.id = 0
        pt_end_marker.type = 7
        pt_end_marker.action = 0
        pt_end_marker.scale.x = 0.02
        pt_end_marker.scale.y = 0.02
        pt_end_marker.scale.z = 0.02
        pt_end_marker.color.r = 1.0
        pt_end_marker.color.g = 0.0
        pt_end_marker.color.b = 0.0
        pt_end_marker.color.a = 1.0

        pt_marker_fail = Marker()
        pt_marker_fail.header.frame_id = "World"
        pt_marker_fail.header.stamp = rospy.get_rostime()
        pt_marker_fail.ns = "World"
        pt_marker_fail.id = 1
        pt_marker_fail.type = 7
        pt_marker_fail.action = 0
        pt_marker_fail.scale.x = 0.02
        pt_marker_fail.scale.y = 0.02
        pt_marker_fail.scale.z = 0.02
        pt_marker_fail.color.r = 1.0
        pt_marker_fail.color.g = 0.0
        pt_marker_fail.color.b = 0.0
        pt_marker_fail.color.a = 1.0

        # reformat the trajectory points
        for count, pt in enumerate(self.traj_pts):
            state = Point()
            state.x = pt[-4]
            state.y = pt[-3]
            traj_states.append(state)
            if float(count) in self.fail_pts:
                pt_marker_fail.colors.append(pt_marker_fail.color)
                pt_marker_fail.points.append(state)
            elif count == 0:
                pt_start_marker.colors.append(pt_start_marker.color)
                pt_start_marker.points.append(state)
            elif count == len(self.traj_pts)-1:
                pt_end_marker.colors.append(pt_end_marker.color)
                pt_end_marker.points.append(state)
            else:
                pt_marker.points.append(state)

        markerPub.publish(pt_marker)
        rospy.sleep(0.1)
        markerPub.publish(pt_start_marker)
        rospy.sleep(0.1)
        markerPub.publish(pt_end_marker)
        rospy.sleep(0.1)
        #print "sending marker"

    def callback(self, data):
        # convert the 1D list to 2D
        if len(self.traj_pts) == 0:
            temp = []
            for count, item in enumerate(data.target):
                if (count + 1) % 4 == 0 and not count == 0:
                    temp.append(item)
                    if not (count+1)/4 in self.fail_pts:
                        self.traj_pts.append(temp)
                    temp = []
                else:
                    temp.append(item)

    def callback1(self, data):
        self.fail_pts.clear()
        print "Adjusting path"
        for item in data.target:
            self.fail_pts.add(item)

    def path_listener(self):
        # listen to the waypoints
        while not rospy.is_shutdown():
            rospy.Subscriber("trajectory_pose", Waypoint, self.callback)
            # rospy.Subscriber("fail_range", Waypoint, self.callback1)
            # call the ik service and print the result generated
            try:
                # visualize the planning success
                self.visualize_result()
            except:
                print "visualizer failed"
            #rospy.sleep(0.1)


if __name__ == "__main__":
    # create a path visualizer object, ros node initialzed
    obj = PathVisualizer()
    print "running visualizer"
    obj.path_listener()
