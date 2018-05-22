#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import rospy

import time, threading

from rcomponent import RComponent, DEFAULT_FREQ, MAX_FREQ 

import robotnik_msgs.msg
import std_msgs.msg


from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *


class PhidgetLoadComponent(RComponent):
    def __init__(self, args):
        RComponent.__init__(self, args)

        self.load_mean_message = std_msgs.msg.Float64()
        self.load_message_c1 = std_msgs.msg.Float64()
        self.load_message_c2 = std_msgs.msg.Float64()
        self.load_message_c3 = std_msgs.msg.Float64()
        self.load_message_c4 = std_msgs.msg.Float64()
        self.load_force_message = std_msgs.msg.Float64()
        self.data = []
        self.buffer_size = 1 #10
        self.data_channels = 4
        self.channels = []
        self.current_index = []
        
        self.gain_channel0=150113 #1.9985 arriba izq
        self.gain_channel1=149522 #2.0064 arriba dcha
        self.gain_channel2=149895 #2.0014 abajo izq
        self.gain_channel3=149723 #2.0037 abajo dcha
        self.gain=[self.gain_channel0,self.gain_channel1,self.gain_channel2,self.gain_channel3]

    def VoltageRatioInputAttached(self, e):
        try:
            attached = e
            '''
            print("\nAttach Event Detected (Information Below)")
            print("===========================================")
            print("Library Version: %s" % attached.getLibraryVersion())
            print("Serial Number: %d" % attached.getDeviceSerialNumber())
            print("Channel: %d" % attached.getChannel())
            print("Channel Class: %s" % attached.getChannelClass())
            print("Channel Name: %s" % attached.getChannelName())
            print("Device ID: %d" % attached.getDeviceID())
            print("Device Version: %d" % attached.getDeviceVersion())
            print("Device Name: %s" % attached.getDeviceName())
            print("Device Class: %d" % attached.getDeviceClass())
            print("\n")
            '''
        except PhidgetException as e:
            rospy.logerr("Phidget Exception %i: %s" % (e.code, e.details))

    def ErrorEvent(self, e, eCode, description):
        rospy.logerr('Phidget error %i: %s' % (eCode, description))

    def VoltageRatioChangeHandler(self, e, voltageRatio):
        # rospy.loginfo("%f %d" % (voltageRatio, e.getChannel()))
        voltageRatio = voltageRatio
        
        
        self.data[e.getChannel()][self.current_index[e.getChannel()]] = voltageRatio*self.gain[e.getChannel()]
       # self.data[e.getChannel()]=voltageRatio*self.gain[e.getChannel()]
        self.current_index[e.getChannel()] = (self.current_index[e.getChannel()] + 1) % self.buffer_size

    def setup(self):
        if self.initialized:
            rospy.logwarn("%s::setup: already initialized" % self.node_name)
            return 0

        RComponent.setup(self)

        rospy.loginfo("%s::setup" % self.node_name)

        for i in range(self.data_channels):
            try:
                ch = VoltageRatioInput()
                ch.setOnAttachHandler(self.VoltageRatioInputAttached)
                ch.setOnErrorHandler(self.ErrorEvent)
                ch.setOnVoltageRatioChangeHandler(self.VoltageRatioChangeHandler)
                ch.setChannel(i)
                ch.openWaitForAttachment(5000)
                if(ch.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE):
                    ch.setBridgeEnabled(1)
                self.channels.append(ch)
                self.data.append([0]*self.buffer_size)
                self.current_index.append(0)
            except RuntimeError as e:
                rospy.logerr('runtime error', e)
                return 0
            except PhidgetException as e:
                rospy.logerr('phidget error: %s, %s', e.code, e.details)
                return 0

    def shutdown(self):
        if self.running:
            rospy.logwarn("%st::shutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.initialized:
            rospy.logwarn("%s::shutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.shutdown(self)

        rospy.loginfo("phidget_load::shutdown")

        for i in range(len(self.channels)):
            self.channels[i].close()

        return 0

    def rosSetup(self):
        if self.ros_initialized:
            rospy.logwarn("%s::rosSetup: already initialized" % self.node_name)
            return 0
        RComponent.rosSetup(self)

        self.load_mean_publisher_ = rospy.Publisher('~load_mean', std_msgs.msg.Float64, queue_size=1)
        self.load_c0_publisher_= rospy.Publisher('~load_c0', std_msgs.msg.Float64, queue_size=1)
        self.load_c1_publisher_= rospy.Publisher('~load_c1', std_msgs.msg.Float64, queue_size=1)
        self.load_c2_publisher_= rospy.Publisher('~load_c2', std_msgs.msg.Float64, queue_size=1)
        self.load_c3_publisher_= rospy.Publisher('~load_c3', std_msgs.msg.Float64, queue_size=1)
        self.load_force_publisher_= rospy.Publisher('~vertical_force', std_msgs.msg.Float64, queue_size=1)

    def rosShutdown(self):
        if self.running:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.ros_initialized:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.rosShutdown(self)

        self.load_mean_publisher_.unregister()
        self.load_c0_publisher_.unregister()
        self.load_c1_publisher_.unregister()
        self.load_c2_publisher_.unregister()
        self.load_c3_publisher_.unregister()
        self.load_force_publisher_.unregister()

    def mean(self, data):
        if len(data) == 0:
            return 0
        return sum(data)/float(len(data))

    def readyState(self):
        data_mean = []
        for i in range(len(self.channels)):
            data_mean.append(self.mean(self.data[i]))

        load_mean = self.mean(data_mean)
        
        self.load_message_c1.data=self.mean(self.data[0])
        self.load_message_c2.data=self.mean(self.data[1])
        self.load_message_c3.data=self.mean(self.data[2])
        self.load_message_c4.data=self.mean(self.data[3])
        self.load_mean_message.data = self.mean(self.data[0])+self.mean(self.data[1])+self.mean(self.data[2])+self.mean(self.data[3])
        self.load_force_message.data = self.mean(self.data[0])-self.mean(self.data[1])+self.mean(self.data[2])-self.mean(self.data[3])

    def rosPublish(self):
        RComponent.rosPublish(self)
        self.load_mean_publisher_.publish(self.load_mean_message)
        self.load_c0_publisher_.publish(self.load_message_c1)
        self.load_c1_publisher_.publish(self.load_message_c2)
        self.load_c2_publisher_.publish(self.load_message_c3)
        self.load_c3_publisher_.publish(self.load_message_c4)
        self.load_force_publisher_.publish(self.load_force_message)
        

def main():
    rospy.init_node("phidget_load")

    _name = rospy.get_name().replace('/','')

    arg_defaults = {
      'topic_state': 'state',
      'desired_freq': DEFAULT_FREQ,
      'port' : '/dev/ttyACM1',
    }

    args = {}

    for name in arg_defaults:
        try:
            if rospy.search_param(name):
                args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, _name))


    phidget_load_node = PhidgetLoadComponent(args)

    phidget_load_node.start()

if __name__ == "__main__":
    main()

