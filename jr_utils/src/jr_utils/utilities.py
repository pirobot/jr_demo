#!/usr/bin/env python

""" utilities.py - Version 1.0 2016-09-11

    Utilities for the JackRabbot project

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

def create_location_markers(marker_scale=0.2, frame_id='map', locations=None, show_labels=True):
    # Define a marker publisher.
    location_marker_pub = rospy.Publisher('location_markers', MarkerArray, queue_size=5)
    
    # Initialize the marker array
    location_markers = MarkerArray()
    
    # Create a marker for each location
    for location in locations:
        location_marker = Marker()
        
        # Fill in the common properties
        marker_lifetime = 0   # 0 is forever
        marker_ns = 'locations'
        marker_color = {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}
    
        location_marker.ns = marker_ns
        location_marker.header.frame_id = frame_id
        location_marker.header.stamp = rospy.Time.now()
        location_marker.type = Marker.CUBE
        location_marker.action = Marker.ADD
        location_marker.lifetime = rospy.Duration(marker_lifetime)
        location_marker.scale.x = marker_scale
        location_marker.scale.y = marker_scale
        location_marker.scale.z = 0.02
        location_marker.color.r = marker_color['r']
        location_marker.color.g = marker_color['g']
        location_marker.color.b = marker_color['b']
        location_marker.color.a = marker_color['a']
        
        location_marker.id = location['id']
        location_marker.pose = location['pose'].pose
        location_marker.text = location['name']
        
        location_markers.markers.append(location_marker)
        
        if show_labels:
            label_marker = Marker()
            label_marker = deepcopy(location_marker)
            label_marker.id = 100 + int(location_marker.id)
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.scale.z = marker_scale
            label_marker.pose.position.x = location_marker.pose.position.x + 0.2
        
            location_markers.markers.append(label_marker)
                
    return location_marker_pub, location_markers

    