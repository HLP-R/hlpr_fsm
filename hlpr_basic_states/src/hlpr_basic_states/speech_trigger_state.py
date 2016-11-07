#!/usr/bin/env python

# Copyright (c) 2016, Diligent Droids
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of hlpr_fsm nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Vivian Chu, vchu@diligentdroids.com

"""speech_trigger_state.py

Based off of the smach_ros.MonitorState class - extends to allow for custom transitions based
on the string value that is being monitored specific to speech
"""
import rospy
import threading
import traceback
import smach

from std_msgs.msg import String
from hlpr_speech_msgs.msg import StampedString
from hlpr_speech_msgs.srv import SpeechService
from hlpr_speech_recognition.speech_listener import SpeechListener

__all__ = ['SpeechTriggerState']

class SpeechTriggerState(smach.State):

    INVALID_KEY = 'invalid'
    PREEMPT_KEY = 'preempted'

    def __init__(self, outcomes=[], input_keys=[], output_keys=[], queue_size=1, forget_ud=False):
        # Extra flag - forget_ud: will clear ud after the state is triggered
        self._forget_ud = forget_ud 
         
        # Get topic that we should be listening to for speech commands
        self._topic = rospy.get_param(SpeechListener.COMMAND_TOPIC_PARAM, None)
        if self._topic is None:
            rospy.logerr("Exiting: No speech topic given, is speech listener running?")
            exit()
       
        # Get message type 
        self._msg_type = eval(rospy.get_param(SpeechListener.COMMAND_TYPE, None))

        # Wait for speech listener
        self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM, None)
        rospy.logwarn("Waiting for speech service")
        rospy.wait_for_service(self.service_topic)
        self.speech_service = rospy.ServiceProxy(self.service_topic, SpeechService)
        rospy.logwarn("Speech service loaded")

        # Check if there exist invalid already
        if SpeechTriggerState.INVALID_KEY not in outcomes:
            # add in the invalid key
            outcomes.append(SpeechTriggerState.INVALID_KEY)

        # Check if there exist invalid already
        if SpeechTriggerState.PREEMPT_KEY not in outcomes:
            # add in the invalid key
            outcomes.append(SpeechTriggerState.PREEMPT_KEY)

        # Standardize all outputs
        outcomes = [item.lower() for item in outcomes]

        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys)

        self._n_checks = 0 
        self._queue_size=queue_size
        self._outcomes = outcomes
        self._trigger_event = threading.Event() 

        self._transition_state = None

    def _cond_cb(self, ud, msg):

        # Pull the speech command
        try:
            response = self.speech_service(True)
            self.last_command = response.speech_cmd.lower()
        except rospy.ServiceException:
            rospy.logerr("No last speech command")
            self.last_command = None

        if self.last_command is None:
            return True

        # Check if the command is within the transitions
        if self.last_command in self._outcomes:
            self._transition_state = self.last_command
            return False
        else:
            rospy.loginfo("Command does not map to state: %s" % self.last_command)
            return True

    def execute(self, ud): 

        # If prempted before even getting a chance, give up. 
        if self.preempt_requested(): 
            self.service_preempt() 
            return SpeechTriggerState.PREEMPT_KEY
 
        self._n_checks = 0 
        self._trigger_event.clear() 

        # Add the callback for the topic provided 
        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=ud, queue_size=self._queue_size) 
 
        self._trigger_event.wait() 
        self._sub.unregister()  # unregister the subscriber after triggering

        if self.preempt_requested(): 
            self.service_preempt() 
            return SpeechTriggerState.PREEMPT_KEY

        if (self._forget_ud):
            # Clear existing userdata
            ud._ud._data = {}

        # Check if we have encoutered a transition state
        if self._transition_state is not None:
            return self._transition_state

        # Continue to spin - "invalid = stay"
        return SpeechTriggerState.INVALID_KEY
 
    def _cb(self,msg,ud) : 
        self._n_checks += 1 
        try: 
            if not self._cond_cb(ud, msg): 
                self._trigger_event.set() 
        except Exception as e: 
            # TODO: Throw a different error now that user is not defining callback
            rospy.logerr("Error thrown while executing condition callback %s: %s" % (str(self._cond_cb), e)) 
            self._trigger_event.set() 

    def request_preempt(self): 
        smach.State.request_preempt(self) 
        self._trigger_event.set() 
