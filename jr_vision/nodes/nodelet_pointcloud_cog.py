#!/usr/bin/env python

"""
    nodelet_pointcloud_cog.py - Version 1.0 2016-10-26
    
    Assume the pointcloud is already filtered using voxelgrid and passthrough filters.
    Publish the COG of the points in a pointcloud falling within a given box.
"""

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

class PointcloudCOG():
    def __init__(self):
        rospy.init_node("pointcloud_cog")
        
        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)
        
        # COG publisher
        self.cog_publisher = rospy.Publisher('nearest_cloud_cog', PointStamped, queue_size=1)

        # Subscribe to the point cloud
        self.depth_subscriber = rospy.Subscriber('point_cloud', PointCloud2, self.pub_cog, queue_size=1)

        rospy.loginfo("Subscribing to point cloud...")
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)

        rospy.loginfo("Ready.")
        
    def pub_cog(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0

        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            x += pt_x
            y += pt_y
            z += pt_z
            n += 1
        
       # If we have points, compute the centroid coordinates
        if n:
            x /= n 
            y /= n 
            z /= n

            cog = PointStamped()
            cog.header.frame_id = msg.header.frame_id
            cog.header.stamp = rospy.Time.now()
            cog.point.x = x
            cog.point.y = y
            cog.point.z = z

            self.cog_publisher.publish(cog)
            
        
    def shutdown(self):        
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)     
                   
if __name__ == '__main__':
    try:
        PointcloudCOG()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pointcloud COG node terminated.")

