#!/usr/bin/env python

"""
    move_person.py - Version 1.0 2017-02-14
    
    Move the base to track an object published on the /target PointStamped topic.
        
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

# Initialize the node
rospy.init_node("move_person")
    
class MovePerson():
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown)
        
        rate = rospy.get_param('~rate', 20)
                
        tick = 1.0 / rate
        
        self.model_states = ModelStates()
        
        # Subscriber to monitor all model states
        rospy.wait_for_message("gazebo/model_states", ModelStates)
        rospy.Subscriber("gazebo/model_states", ModelStates, self.get_model_states)

        # Publisher for moving the robot
        model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=5)
        
        # Initialize the people model states
        person_1 = ModelState()
        person_2 = ModelState()
         
        # Get the indices of the two people in the ModelStates array
        while len(self.model_states.name) == 0:
            rospy.sleep(0.5)
        
        person_1_index = self.model_states.name.index('person_walking')
        person_2_index = self.model_states.name.index('person_walking_0')
        
        person_1.model_name = 'person_walking'
        person_1.reference_frame = 'world'
        person_1.pose = self.model_states.pose[person_1_index]
        person_1_start_y = person_1.pose.position.y
        person_1_start_orientation = person_1.pose.orientation

        person_2.model_name = 'person_walking_0'
        person_2.reference_frame = 'world'
        person_2.pose = self.model_states.pose[person_2_index]
        person_2_start_x = person_2.pose.position.x
        person_2_start_orientation = person_2.pose.orientation
                         
        rospy.loginfo("Starting person mover.")
                                 
        while not rospy.is_shutdown():
            while (person_1.pose.position.y - -2.0) > 0.1:
                person_1.pose.position.y -= 0.02
                
                if (person_2.pose.position.x - -7.0) > 0.1:
                    person_2.pose.position.x -= 0.02

                model_state_pub.publish(person_1)
                model_state_pub.publish(person_2)
                
                rospy.sleep(tick)
                
                #person_1.pose = self.model_states.pose[person_1_index]
                #person_2.pose = self.model_states.pose[person_2_index]
                
            rpy = euler_from_quaternion([person_1.pose.orientation.x, person_1.pose.orientation.y, person_1.pose.orientation.z, person_1.pose.orientation.w])
            yaw = rpy[2]
            yaw += math.pi
            
            quat = quaternion_from_euler(rpy[0], rpy[1], yaw)
            
            person_1.pose.orientation.x = quat[0]
            person_1.pose.orientation.y = quat[1]
            person_1.pose.orientation.z = quat[2]
            person_1.pose.orientation.w = quat[3]
            
            rpy = euler_from_quaternion([person_2.pose.orientation.x, person_2.pose.orientation.y, person_2.pose.orientation.z, person_2.pose.orientation.w])
            yaw = rpy[2]
            yaw += math.pi
            
            quat = quaternion_from_euler(rpy[0], rpy[1], yaw)
            
            person_2.pose.orientation.x = quat[0]
            person_2.pose.orientation.y = quat[1]
            person_2.pose.orientation.z = quat[2]
            person_2.pose.orientation.w = quat[3]
    
            while abs(person_1.pose.position.y - person_1_start_y) > 0.1:
                person_1.pose.position.y += 0.02
                
                if abs(person_2.pose.position.x - person_2_start_x) > 0.1:
                    person_2.pose.position.x += 0.02
                    
                model_state_pub.publish(person_1)
                model_state_pub.publish(person_2)
                
                rospy.sleep(tick)
                
                #person_1.pose = self.model_states.pose[person_1_index]    
            
            person_1.pose.orientation = person_1_start_orientation
            person_2.pose.orientation = person_2_start_orientation    
   
            
    def get_model_states(self, msg):
        self.model_states = msg
                
    def shutdown(self):
        rospy.loginfo("Shutting down move person node...")
                   
if __name__ == '__main__':
    try:
        MovePerson()
    except rospy.ROSInterruptException:
        rospy.loginfo("Move person node terminated.")




