#!/usr/bin/env python

"""
    jr_speech_module.py - Version 1.0 2016-09-10
    
    Look for people published on the people detection topic and offer to usher them
    to specific locations within a conference hall.
    
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
import os
import actionlib
from jr_msgs.msg import Location
#from jr_msgs.srv import GotoLocationRequest, GotoLocationResponse, GotoLocation
from cob_perception_msgs.msg import DetectionArray
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from std_srvs.srv import Empty
from yaml import load
from math import sqrt
import random

class SpeechModule():
    def __init__(self):
        rospy.init_node("jr_speech_module")
        
        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param('~rate', 10)
                
        self.tick = 1.0 / self.rate
        
        # How close does a person have to be to pay attention to them?
        self.max_target_distance = rospy.get_param('~max_target_distance', 2.0)
        
        # Set the default TTS voice to use
        self.tts_voice = rospy.get_param("~tts_voice", "voice_don_diphone")
        
        # How long in seconds before we consider a detection lost?
        self.lost_detection_timeout = rospy.get_param('~lost_detection_timeout', 5.0)
        
        # Get the path of the navigation configuration file
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
        
        # Are we lisenting on the speech recognition topic?
        self.listening = False
        
        # Set a timer to determine how long a target is no longer visible
        self.target_lost_time = 0.0
        
        # Create the sound client object
        self.soundhandle = SoundClient(blocking=True)
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(2)
        
        # Publish the requested location so the executive node can use it
        self.location_pub = rospy.Publisher('/speech_navigation', String, queue_size=1)

        # Connect to the goto_location service
        #rospy.wait_for_service('/goto_location', 60)
 
        #self.goto_service = rospy.ServiceProxy('/goto_location', GotoLocation)

        # Subscribe to the speech recognition /recognizer/output topic to receive voice commands
        rospy.Subscriber("/recognizer/output", String, self.speech_recognition, queue_size=1)
        
        # Subscribe to the target topic for tracking people
        rospy.Subscriber('target_topic', DetectionArray, self.detect_person, queue_size=1)
        
        # Wait for the speech recognition services to be ready
        rospy.wait_for_service('/recognizer/start', 15)
        rospy.wait_for_service('/recognizer/stop', 15)
        
        # Connect to the start/stop services for speech recognition
        self.stop_speech_recognition = rospy.ServiceProxy('/recognizer/stop', Empty)
        self.start_speech_recognition = rospy.ServiceProxy('/recognizer/start', Empty)
        
        rospy.loginfo("JR demo up and running.")
        
        # Announce that we are ready for input
        self.jr_says("Ready", self.tts_voice)
        self.jr_says("Say, hello jack rabbit, to get my attention", self.tts_voice)
        
        #self.start_speech_recognition()
        
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
            
    def jr_says(self, text, voice, start_listening=False, pause=2):
        self.listening = False
        self.soundhandle.say(text, voice)
        if start_listening:
            rospy.sleep(2)
            self.listening = True

            
    def detect_person(self, msg):
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

    def greet_person(self):
        if not self.greeting_finished:
            self.jr_says("Hello there.  My name is Jack Rab bot.", self.tts_voice)
            self.jr_says("Thanks for coming to the conference.", self.tts_voice)
            self.jr_says("Say, hello jack rabbit, to get my attention", self.tts_voice)

            #self.jr_says("I like what you are wearing.", self.tts_voice)
            #self.jr_says("Where would you like to go?", self.tts_voice)
            self.greeting_finished = True
                
    def speech_recognition(self, msg):
        # Look for a wake up phrase
        if msg.data in ['hey jr', 'hi jr', 'hey jackrabbot', 'hi jackrabbot', 'hey jackrabbit', 'hi jackrabbit', 'hello jackrabbot', 'hello jackrabbit']:
            self.jr_says("Hello there. Where would you like to go?", self.tts_voice, start_listening=True)
            return

        if not self.listening:
            return
        
        if msg.data in ['poster session', 'poster sessions', 'poster', 'posters', 'poster area']:
            location = "posters"
        elif msg.data in ['keynote session', 'keynote sessions', 'keynote talk', 'keynote talks', 'keynote', 'keynotes']:
            location = "keynotes"
        elif msg.data in ['demo', 'demos', 'demonstration', 'demonstrations']:
            location = "demos"
        elif msg.data in ['tutorial', 'tutorials', 'tutorial session']:
            location = "tutorials"
        elif msg.data in ['exhibit', 'exhibits', 'exhibit area']:
            location = "exhibits"
        elif msg.data in ['washroom', 'washrooms', 'restroom', 'restrooms', 'bathroom', 'bathrooms', 'mens washroom', 'mens restroom', 'mens bathroom', 'womens washroom', 'womens restroom', 'womens bathroom']:
            location = "restrooms"
        elif msg.data in ['food', 'food area', 'food truck', 'food trucks', 'something to eat', 'cafeteria', 'to trucks', 'to food']:
            location = "food"
        elif msg.data in ['entrance', 'foyer']:
            location = "entrance"
        elif msg.data in ['exit']:
            location = "entrance"
        else:
            self.jr_says("I'm sorry. I did not understand that. Please say again?", self.tts_voice)
            return
        
        self.begin_phrases = ['Great choice.', 'No problem.', 'Sure thing.', 'My pleasure.']
        self.end_phrases = ['Right this way.', 'Please follow me.', 'Come this way.']
       
        location_to_phrase = {'posters':'poster sessions',
                              'keynotes':'keynote talks',
                              'demos':'demonstrations', 
                              'exhibits':'exhibits',
                              'tutorials':'tutorials',
                              'restrooms':'restrooms',
                              'mens restroom':'mens restroom',
                              'womens restroom':'womens restroom',
                              'food':'food trucks',
                              'entrance':'entrance',
                              'exit':'exit'}
        
        random.shuffle(self.begin_phrases)
        random.shuffle(self.end_phrases)
        
        # Randomize the beginning and ending of each response
        if location == 'restrooms':
            begin = 'No problem'
        else:
            begin = self.begin_phrases[0]
            
        end = self.end_phrases[0]
        
        response = begin + ' I will take you to the ' + location_to_phrase[location] + '. ' + end
        
        self.jr_says(response, self.tts_voice)
        
        # Publish the request
        nav_request = String()
        nav_request.data = location
        self.location_pub.publish(nav_request)
        
        # Create a goto request for the navigation server
        #request = GotoLocationRequest()
        #request.location.name = location
 
        #response = self.goto_service(request)
        
        rospy.loginfo("Speech navigation: " + str(location))
        #rospy.loginfo(response)
        
        #if response.success:
        #   self.jr_says("We have arrived.", self.tts_voice)
            
    def shutdown(self):
        rospy.sleep(1)
        os._exit(0)

if __name__ == '__main__':
    SpeechModule()

