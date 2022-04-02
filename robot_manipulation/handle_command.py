#!/usr/bin/env python

from __future__ import print_function
from tkinter.messagebox import YES

from click import command
from xf_mic_asr_offline.srv import VoiceCommand, VoiceCommandResponse
import argparse as ap
from std_msgs.msg import String, Bool
import rospy
from handover_object import HandoverObjectNode as hdo
from touch_object import TouchObjectNode as hto
from greetings import greetingsNode as gn

class HandleCommandNode():
    def __init__(self):
        self.node_name = 'HandleCommandNode'
        self.rate = 10.0
        self.task_id = None

    def handle_voice_command(self, req):
        #voice_command_map = {"关一下灯": 0, '开一下灯': 1, '拿一下水': 2, '我是谁':3}
        if (req.task_id == 0):
            result = self.handle_hand_over_object()
        elif (req.task_id == 1):
            result= self.handle_hand_over_object()
        elif (req.task_id == 2):
            result = self.handle_hand_over_object('water')
        elif (req.task_id == 3):
            result = self.handle_greetings()
        # return result to Cpp voice interface
        if result == True:
            ans = VoiceCommandResponse()
            ans.successful = True
            ans.redo_required = False
            ans.message = "任务完成"
        else:
            ans = VoiceCommandResponse()
            ans.successful = False
            ans.redo_required = True
            ans.message = "任务失败"
        return ans

    def handle_greetings(self, contents):
        task_node = gn()
        result = task_node.main() 
        return result

    def handle_touch_object(self, object_name):
        task_node = hto()
        task_node.set_object_name(object_name)
        result = task_node.main() 
        return result

    def handle_hand_over_object(self, object_name):
        task_node = hdo()
        task_node.set_object_name(object_name)
        result = task_node.main() 
        return result

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