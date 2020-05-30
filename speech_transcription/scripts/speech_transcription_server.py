#!/usr/bin/env python
import rospy
import speech_recognition as sr

from speech_transcription.srv import InquireColor, InquireColorResponse
from speech_transcription.srv import InquireAffirmation, InquireAffirmationResponse
from speech_transcription.srv import InquirePreferences, InquirePreferencesResponse

class SpeechTranscriptionServer(object):

    def __init__(self):
        rospy.init_node('speech_transcription')

        self.colors = ['red', 'green', 'blue', 'yellow', 'white', 'black']

        # Create a service for processing requests about women's favorite color
        color_service = rospy.Service('inquire_color', InquireColor, self.handle_color_inquiry_request)
        # Service for processing requests about Gargamel's preferences
        preferences_service = rospy.Service('inquire_preferences', InquirePreferences, self.handle_preferences_inquiry_request)
        # Service for processing affirmations from Gargamel and women
        affirmation_service = rospy.Service('inquire_affirmation', InquireAffirmation, self.handle_affirmation_inquiry_request)

    def listen(self):
        # Create a Recognizer() object that is used for transcribing speech
        recognizer = sr.Recognizer()
        # Create Microphone object, that uses the device's default microphone
        mic = sr.Microphone()

        rospy.loginfo('Listening for speech ...')

        # Adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        with mic as source:
            #recognizer.adjust_for_ambient_noise(source, duration=1)
            audio = recognizer.listen(source)
        
        response = {
            'success': True,
            'error': None,
            'transcription': None
        }

        # response2 = type('obj', (object,), {
        #     'success': True,
        #     'error': None,
        #     'transcription': None
        # })

        try:
            response['transcription'] = recognizer.recognize_google(audio)
        except sr.RequestError:
            # API was unreachable or unresponsive
            response['success'] = False
            response['error'] = 'API unavailable'
        except sr.UnknownValueError:
            # Speech was unrecognisable
            response['error'] = "Unable to recognise speech"
        
        return response

    def handle_preferences_inquiry_request(self, request):
        """This method handles speech recognition requests about Gargamel's preferences for women.
        Returns hair_length and hair_color"""
        response = self.listen()
        transcription = response['transcription']

        if 'bright' in transcription:
            hair_color = 'bright'
        elif 'dark' in transcription:
            hair_color = 'dark'
        
        if 'short' in transcription:
            hair_length = 'short'
        elif 'long' in transcription:
            hair_length = 'long' 

        return InquirePreferencesResponse(hair_length, hair_color)
    
    def handle_affirmation_inquiry_request(self, request):
        """This method handles speech recognition requests that need affirmation (yes/no answers)"""
        response = self.listen()
        transcription = response['transcription']

        if 'yes' in transcription:
            affirmation = 'yes'

        elif 'no' in transcription:
            affirmation = 'no'

        else:
            affirmation = 'error'

        return InquireAffirmationResponse(affirmation)
    
    def handle_color_inquiry_request(self, request):
        """This method handles speech recognition requests about color."""
        response = self.listen()
        transcription = response['transcription']

        favorite_color = ''

        for color in self.colors:
            if color in transcription:
                favorite_color = color

        return InquireColorResponse(favorite_color)

if __name__ == '__main__':
    speech_transcriptor = SpeechTranscriptionServer()
    rospy.loginfo("Speech transcription node started")
    rospy.spin()