#!/usr/bin/env python

"""
    jr_demo.py - Version 1.0 2016-09-10
    
    Look for people and offer to usher them to specific locations within a conference hall.
    
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
import actionlib
from jr_msgs.msg import Location
from jr_msgs.srv import GotoLocationRequest, GotoLocationResponse, GotoLocation
from cob_perception_msgs.msg import DetectionArray
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from yaml import load
from math import sqrt

class JRDemo():
    def __init__(self):
        rospy.init_node("jr_demo")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param('~rate', 10)
                
        self.tick = 1.0 / self.rate
        
        self.max_target_distance = rospy.get_param('~max_target_distance', 2.0)
        
        # Set the default TTS voice to use
        #self.tts_voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")
        self.tts_voice = rospy.get_param("~tts_voice", "voice_don_diphone")
        
        # How long before we consider a detection lost?
        self.lost_detection_timeout = rospy.get_param('~lost_detection_timeout', 5.0)
        
        # Get the path of the configuration file
        nav_config_file = rospy.get_param('~nav_config_file', 'config/locations.yaml')
        
        # Load the location data
        with open(nav_config_file, 'r') as config:
            self.locations = load(config)
        
        # Have we said the greeting to a new person?
        self.greeting_finished = False
        
        # Is a person visible?
        self.target_visible = False
        
        # Are we waiting for the next detection?
        self.waiting = False
        
        # Set a timer to determine how long a target is no longer visible
        self.target_lost_time = 0.0
        
        # Create the sound client object
        self.soundhandle = SoundClient(blocking=True)
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.say("Ready", self.tts_voice)
        rospy.sleep(2)

#         # Connect to the goto_location service
#         rospy.wait_for_service('/goto_location', 60)
# 
#         goto_service = rospy.ServiceProxy('/goto_location', GotoLocation)
#         
#         request = GotoLocationRequest()
#         request.location.name = "bathroom"
# 
#         response = goto_service(request)
#         rospy.loginfo(response)

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber("/recognizer/output", String, self.speech_recognition)
        
        # Subscribe to the target topic for tracking people
        rospy.Subscriber('target_topic', DetectionArray, self.greet_person, queue_size=1)
        
        rospy.loginfo("JR demo up and running.")  
        
        while not rospy.is_shutdown():
            # If we have lost the target, start a timer
            if not self.target_visible:
                self.target_lost_time += self.tick
                
                if self.target_lost_time > self.lost_detection_timeout and not self.waiting:
                    rospy.loginfo("No person in sight.")
                    self.target_visible = False
                    self.greeting_finished = False
                    self.waiting = True
            else:
                if self.waiting:
                    rospy.loginfo("Person detected.")
                    self.waiting = False
                    self.target_lost_time = 0.0

            rospy.sleep(self.tick)
                             
#          
#         # Create the "stay healthy" selector
#         #STAY_HEALTHY = Selector("STAY_HEALTHY")
#          
#         # Create the patrol loop decorator
#         LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols)
#          
#         # Add the two subtrees to the root node in order of priority
#         #BEHAVE.add_child(STAY_HEALTHY)
#         BEHAVE.add_child(LOOP_PATROL)
#          
#         # Create the patrol iterator
#         PATROL = Iterator("PATROL")
#          
#         IGNORE_FAILURE = IgnoreFailure("IGNORE_FAILURE")
#          
#         IGNORE_FAILURE.add_child(PATROL)
#          
#         # Add the move_base tasks to the patrol task
#         for task in MOVE_BASE_TASKS:
#             PATROL.add_child(task)
#    
#         # Add the patrol to the loop decorator
#         LOOP_PATROL.add_child(PATROL)
#          
#         # Add the battery check and recharge tasks to the "stay healthy" task
#         with STAY_HEALTHY:
#             # The check battery condition (uses MonitorTask)
#             CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
#             
#             # The charge robot task (uses ServiceTask)
#             CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
#       
#             # Build the recharge sequence using inline construction
#             RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
#                 
#             # Add the check battery and recharge tasks to the stay healthy selector
#             STAY_HEALTHY.add_child(CHECK_BATTERY)
#             STAY_HEALTHY.add_child(RECHARGE)
                 
        # Display the tree before beginning execution
        #print "Patrol Behavior Tree"
        #print_tree(BEHAVE, use_symbols=True)
         
        # Run the tree
#         while not rospy.is_shutdown():
#             #BEHAVE.run()
# 
#             rospy.sleep(0.1)
            
    def greet_person(self, msg):
        min_distance = 10000
        target_head = None
        self.target_visible = False
                
        # Pick the closest detection
        for head in msg.detections:
            pos = head.pose.pose.position
            distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z + pos.z)
            if distance < min_distance and distance < self.max_target_distance:
                target_head = head
        
        if target_head is not None:
            # Set detection flag
            self.target_visible = True
            
            # Greet person
            if not self.greeting_finished:
                self.soundhandle.say("Hello there.  My name is Jack Rab bot.", self.tts_voice)
                #self.soundhandle.say("Thanks for coming to the conference.", self.tts_voice)
                #self.soundhandle.say("I like what you are wearing.", self.tts_voice)
                #self.soundhandle.say("Where would you like to go?", self.tts_voice)
                self.greeting_finished = True
                
    def speech_recognition(self, msg):
        if "poster" in msg.data:
            self.soundhandle.say("OK.  I will take you to the poster sessions.  Please follow me.", self.tts_voice)
        elif "keynote" in msg.data:
            self.soundhandle.say("Great choice.  I will take you to the keynote sessions.  Right this way.", self.tts_voice)
        else:
            self.soundhandle.say("I'm sorry.  I did not understand that.  Please say again?", self.tts_voice)
        
        return
            
    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    tree = JRDemo()

