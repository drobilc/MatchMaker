#!/usr/bin/env python

import rospy
import sound_play
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Object that abstracts voice commands
class Greeter(object):

    FACE_GREETINGS = [
        "Hello there!", "How are you doing?", "Oh, hi", "Good day", "Hello"
    ]
    
    OBJECT_GREETINGS = [
        "What a beautiful {} {}!",
        "Well, this is a nice {} {}.",
        "I see a {} {}.",
        "My senses tell me this is a {} {}.",
        "I declare this a {} {}.",
        "{} {}, not suprising.",
    ]

    def __init__(self):
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0
        self.client = SoundClient()
        rospy.loginfo("Greeter created!")
    
    def say(self, data):
        # Send data to the client, client outputs the data with given voice and volume
        rospy.loginfo("Send to sound_client: " + data)
        self.client.say(data, self.voice, self.volume)
    