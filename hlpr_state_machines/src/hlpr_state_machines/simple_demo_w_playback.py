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

""" simple_demo_w_playback.py

Basic state machine that playsback a previously recorded demonstration
"""

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from hlpr_basic_states.speech_trigger_state import SpeechTriggerState
from hlpr_speech_recognition.speech_listener import SpeechListener
from hlpr_record_demonstration.MoveitPlaybackBagState import MoveitPlaybackBagState


def main():

    # initialize the rosnode
    rospy.init_node('simple_demo_playback_fsm')

    # Set for whether we want to use speech
    use_speech = True

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['DONE']) # only hit done if we're preempted

    ROSBAG_PARAM_NAME = "~bagfile"

    task1 = "~/data/demo_01.bag"
    task2 = "~/data/demo_06.bag"

    # Open the container
    with sm:

        smach.StateMachine.add('IDLE', SpeechTriggerState(outcomes=['playback', 'start_exp'], output_keys=['bag_file', 'eef_flag','target_topic']), transitions={'invalid': 'IDLE', 'preempted':'DONE', 'playback':'TASK1', 'start_exp':'TASK2'})
        smach.StateMachine.add('TASK1', MoveitPlaybackBagState(bagfile=task1, eef_flag=False, pose_topic='joint_states'), transitions={'playback_finished':'IDLE'})
        smach.StateMachine.add('TASK2', MoveitPlaybackBagState(bagfile=task2, eef_flag=True, pose_topic='eef_pose'), transitions={'playback_finished':'IDLE'})

    # Execute SMACH plan
    outcome = sm.execute()




if __name__ == '__main__':
    main()
