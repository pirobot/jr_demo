#!/usr/bin/env python

"""
    say_phrases.py - Version 1.1 2013-12-20
    
    Use the sound_play client play text-to-speech or pre-recorded wav files.
    
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
    
    http://www.gnu.org/licenses/gpl.htmlPoint
"""

import rospy
import sys, os
from jr_msgs.srv import SayPhrase, SayPhraseResponse
from sound_play.libsoundplay import SoundClient
from yaml import load
from random import sample, randint

class SayPhrases:
    def __init__(self, script_path):
        rospy.init_node("say_phrases")

        rospy.on_shutdown(self.cleanup)
        
        # Get the path of the configuration file
        config_file = rospy.get_param("~config_file", None)
        
        # Use text-to-speech or play wave files?
        self.use_tts = rospy.get_param("~use_tts", True)
        
        # Play random phrases or listen on a service for a signal?
        self.random_phrases = rospy.get_param("~random_phrases", True)
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../wave_files")
        
        self.phrases = rospy.get_param("~phrases", None)
        
        # Load the configuration parameters
        with open(config_file, 'r') as config:
            self.phrases = load(config)
                
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)
        rospy.sleep(2)
        
        if not self.random_phrases:
            # Create a service to accept speech requests
            rospy.Service("~say_phrase", SayPhrase, self.SayPhraseHandler)
            rospy.loginfo("Waiting for signal to say something...")
            rospy.spin()
        else:
            rospy.loginfo("Saying random phrases...")
            
            while not rospy.is_shutdown():
                phrases = sample(self.phrases, len(self.phrases))
                
                for phrase in phrases:
                    if self.use_tts:
                        self.soundhandle.say(str(phrase['phrase']), self.voice)
                    else:
                        if phrase['file'] is not None:
                            self.soundhandle.playWave(self.wavepath + '/' + phrase['file'])
                        else:
                            self.soundhandle.say(str(phrase['phrase']), self.voice)
                       
                    rospy.sleep(randint(2, 10))
        
    def SayPhraseHandler(self, req):
        id = req.id.data
        
        rospy.loginfo(id)
            
        if id > len(self.phrases) - 1:
            rospy.loginfo("Phrase ID out of bounds. Max possible value is " + str(len(self.phrases)))
            return
                
        [phrase] = [phrase for phrase in self.phrases if phrase['id'] == id]
        
        if self.use_tts:
            self.soundhandle.say(phrase['phrase'], self.voice)
        else:
            if phrase['file'] is not None:
                self.soundhandle.playWave(self.wavepath + '/' + phrase['file'])
            else:
                self.soundhandle.say(str(phrase['phrase']), self.voice)
                
        return SayPhraseResponse()

    def cleanup(self):
        try:
            self.soundhandle.stopAll()
            os._exit(0)
        except:
            pass
        rospy.loginfo("Shutting down say phrases node...")

if __name__=="__main__":
    try:
        SayPhrases(sys.path[0])
    except rospy.ROSInterruptException:
        rospy.loginfo("Say phrases node terminated.")
