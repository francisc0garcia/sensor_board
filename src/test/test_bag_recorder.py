#!/usr/bin/env python

from src.classes.bags.recorder import *

import rospy
import rosbag
import time
import threading

from sensor_board.srv import record_msg

class BagRecorder:
    def __init__(self):
        self._recorder = None
        self.filename = '/home/pach0/Documents/HiWi_Control_systems/code/data/test.bag'
        self.all = False
        self.regex = False
        self.limit = 0
        self.topics=[] # '/gpsMsg', '/sensor/imu_2','/sensor/imu_1'
        self._bag_lock = threading.RLock()

        self.service_control = rospy.Service('change_bag_record', record_msg, self.callback_service)

        self.enable_recording = True

        #self._recorder.add_listener(self._message_recorded)
        #def _message_recorded(self, topic, msg, t):

        self.rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.rate.sleep()

        rospy.loginfo('Stop recording')

    def callback_service(self, req):
        # print req.topics
        # print req.enable_record
        state = ''
        self.topics = req.topics
        self.filename = req.filename
        self.enable_recording = req.enable_record

        if self.enable_recording:
            try:
                self._recorder = Recorder(self.filename,
                                          bag_lock = self._bag_lock,
                                          all = self.all,
                                          topics = self.topics,
                                          regex = self.regex,
                                          limit = self.limit)
                self._recorder.start()
                state = 'recording'
            except Exception as ex:
                rospy.loginfo('Error opening bag ')
                state = 'error ' + ex
        else:
            self._recorder.stop()
            state = 'stopped'

        rospy.loginfo('state bag recording: ' + state)
        return 'state bag recording: ' + state

# Main function.
if __name__ == '__main__':
    rospy.init_node('test_bags', anonymous=True)
    obj = BagRecorder()