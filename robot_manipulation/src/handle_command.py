#!/usr/bin/env python

from __future__ import print_function

from click import command
from xf_mic_asr_offline.srv import VoiceCommand, VoiceCommandResponse
import argparse as ap
from std_msgs.msg import String, Bool
import rospy

class HandleCommandNode():
    def __init__(self):
        self.node_name = 'HandleCommandNode'
        self.rate = 10.0
        self.joint_states = None

    def handle_voice_command(self, req):
        


        ans = VoiceCommandResponse()
        ans.successful = True
        ans.redo_required = False
        ans.message = "任务完成"
        return ans

    def main(self):
        rospy.init_node(self.node_name)
        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))
        # rosservice for receivng command from voice control and give feedback
        command_handler_service = rospy.Service('voice_command', VoiceCommand, self.handle_voice_command)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Handle and analyse vocal commands.')
        args, unknown = parser.parse_known_args()
        node = HandleCommandNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')