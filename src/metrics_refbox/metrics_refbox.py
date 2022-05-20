#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import rosgraph
import std_msgs.msg
import os
import json
import uuid
import datetime
import metrics_refbox_msgs.msg
from metrics_refbox_msgs.msg import Command, Confirm


class MetricsRefbox(object):
    """
    ROS node for refbox; handles communication with refbox client on the robot
    """
    def __init__(self, status_cb, start_cb, result_cb):
        self.robot_event_out = None
        self.rosbag_event_out = None
        self.rosbag_filename = None
        config_file = rospy.get_param('~config_file')
        self.config_file_path = rospy.get_param('~config_file_path')
        self.results_file_path = rospy.get_param('~results_file_path')

        stream = open(os.path.join(self.config_file_path, config_file), 'r')
        self.config = json.load(stream)

        self.gui_start_cb = start_cb
        self.gui_result_cb = result_cb
        self.gui_status_cb = status_cb
        self.master = rosgraph.Master("")

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.command_msg_pub = rospy.Publisher("~command", metrics_refbox_msgs.msg.Command, queue_size=1)
        self.test_pub = rospy.Publisher('/test_topic', std_msgs.msg.String, queue_size=1)

        # Subscribers
        rospy.Subscriber("~command_confirmation", metrics_refbox_msgs.msg.Confirm, self.command_confirmation_cb)
        for key in self.config['benchmarks'].keys():
            msg_type = getattr(metrics_refbox_msgs.msg, self.config['benchmarks'][key]['type'])
            rospy.Subscriber("~%s" % self.config['benchmarks'][key]['topic'], msg_type, self.result_cb)
        rospy.Subscriber('/test_topic_ack', std_msgs.msg.String, self.test_topic_cb)

    def command_confirmation_cb(self, msg):
        '''
        callback for command confirmation message from refbox client
        '''
        if msg.uid != self.last_command_uid:
            self.gui_status_cb("confirmation has different uid\n")
            return
        self.gui_start_cb(msg)

    def result_cb(self, msg):
        '''
        callback for all type of result messages from refbox client
        '''
        self.gui_result_cb(msg)

    def get_config(self):
        '''
        return refbox config as dictionary
        '''
        return self.config

    def get_config_file_path(self):
        '''
        return path to refbox config file
        '''
        return self.config_file_path

    def get_results_file_path(self):
        '''
        return path to results folder
        '''
        return self.results_file_path

    def start(self, benchmark_enum, benchmark_config):
        '''
        send start command to refbox client
        '''
        self.last_command_uid = str(uuid.uuid4())
        msg = Command()
        msg.uid = self.last_command_uid
        msg.command = Command.START
        msg.task = benchmark_enum
        msg.task_config = json.dumps(benchmark_config)
        self.command_msg_pub.publish(msg)

    def start_recording(self):
        self.last_command_uid = str(uuid.uuid4())
        msg = Command()
        msg.uid = self.last_command_uid
        msg.command = Command.START
        msg.task = 0
        self.command_msg_pub.publish(msg)

    def stop(self):
        '''
        send stop command to refbox client
        '''
        msg = Command()
        msg.command = Command.STOP
        self.command_msg_pub.publish(msg)

    def test_communication(self):
        '''
        send test message to refbox client
        '''
        self.test_pub.publish('ping')

    def test_topic_cb(self, msg):
        '''
        receive reply to test message from refbox client
        '''
        node_ip = self.master.lookupNode(msg._connection_header['callerid'])
        status_msg = 'Received reply from ' + msg.data + ' at ' + node_ip + '\n'
        self.gui_status_cb(status_msg)
