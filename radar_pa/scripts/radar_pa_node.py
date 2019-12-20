#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Radar_Pa package, this is radar_pa_node """

# *****************************************************************************
#                                                                             *
# radar_pa_node.py                                                            *
#                                                                             *
# *****************************************************************************
#                                                                             *
# github repository                                                           *
#     https://github.com/TUC-ProAut/radar_pa                                  *
#                                                                             *
# Chair of Automation Technology, Technische Universität Chemnitz             *
#     https://www.tu-chemnitz.de/etit/proaut                                  *
#                                                                             *
# *****************************************************************************
#                                                                             *
# BSD 3-Clause License                                                        *
#                                                                             *
# Copyright (c) 2018-2019, Karim Haggag, Technische Universität Chemnitz      *
# All rights reserved.                                                        *
#                                                                             *
# Redistribution and use in source and binary forms, with or without          *
# modification, are permitted provided that the following conditions are met: *
#     * Redistributions of source code must retain the above copyright        *
#       notice, this list of conditions and the following disclaimer.         *
#     * Redistributions in binary form must reproduce the above copyright     *
#       notice, this list of conditions and the following disclaimer in the   *
#       documentation and/or other materials provided with the distribution.  *
#     * Neither the names of the copyright holders nor the names of its       *
#       contributors may be used to endorse or promote products               *
#       derived from this software without specific prior written permission. *
#                                                                             *
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE  *
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         *
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        *
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    *
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     *
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     *
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  *
# POSSIBILITY OF SUCH DAMAGE.                                                 *
#                                                                             *
# *****************************************************************************
#                                                                             *
# Revision 1                                                                  *
#                                                                             *
# *****************************************************************************
#                                                                             *
# This node converts the given data from the can messages to radar_msgs.      *
#                                                                             *
# Supported radars:                                                           *
#     * Bosch GPR v1.0                                                        *
#                                                                             *
# *****************************************************************************

# -- ros modules
import rospy
import std_msgs.msg
from can_msgs.msg       import Frame
from radar_pa_msgs.msg  import radar_msg_A
from radar_pa_msgs.msg  import radar_msg_B
from radar_pa_msgs.msg  import radar_msg
from extract_message    import extract_GPR_v10

# -- this is a class now :-)
class RadarPaNode:

    """This is a Radar Node class, which instansiate a node and
    subscribig to received_messages topice to be able to read can messages.
    Then, publish the extracted message A on "can_messages_A", extracted
    message A on "can_messages_A" and both A,B on "radar_messages"."""

    def __init__(self):

        # -- instantiat instance  a node
        rospy.init_node('radar_pa_node.py', anonymous=True)

        # -- use socketcan_bridge package /  then subscribe to the topic 'received_messages'
        # -- published by socketcan_bridge_node
        # -- we install 'socketcan_bridge package' so it becomes system package
        #    /opt/ros/kinetic/share/socketcan_bridge
        rospy.Subscriber('received_messages', Frame, self.callback)

        # -- publish radar_msg_A type
        self.pub_a = rospy.Publisher('can_messages_A', radar_msg_A, queue_size=10)
        # -- radar_msg_B type
        self.pub_b = rospy.Publisher('can_messages_B', radar_msg_B, queue_size=10)
        # -- both radar_msg A,B
        self.pub   = rospy.Publisher('radar_messages', radar_msg  , queue_size=10)

        # -- instantiate radar_messages
        self.global_msg = radar_msg()

        # -- instantiate radar_message
        self.global_msg.header = std_msgs.msg.Header()

        # -- assign frame id
        self.global_msg.header.frame_id = "radar"

        # -- set time stamp
        self.global_msg.header.stamp = rospy.Time.now()

    def callback(self, data):

        """This method calling the extection function and fill the msg with
         the result from the extraction function."""
        self.global_msg.header.stamp = data.header.stamp

        # -- check the bounderies of id
        if (data.id >= 512 and data.id <= 607):

            # -- call the extract function from extract_message file
            msg = extract_GPR_v10(data.data)

            # -- call fill_message
            self.fill_message(data, msg)
            # -- check message type the publish to the corresponding topic
            if (isinstance(msg, radar_msg_A)):
            #if (msg.message == 1) :
                self.pub_a.publish(msg)

            else:
                self.pub_b.publish(msg)

            # -- check if the message have the same counter then publish it
            if (data.id == 607):
                if (self.check_message_counter() is True):#== True):
                    rospy.loginfo('the whole packet is completed')
                    self.pub.publish(self.global_msg)

        else:
            rospy.loginfo('can id out of range %s', hex(data.id))

    # ***************** fill the message with msg A , B  *****************

    def fill_message(self, data, msg):

        """ Ths function to fill the global message with the extracted result"""
        if (data.id % 2 == 0):
            # -- can_messages_A start from 0x200 = 512
            i = (data.id - 512)/2
            self.global_msg.data_A[i] = msg
        else:
            # -- can_messages_A start from 0x201 = 513
            j = (data.id - 513) / 2
            self.global_msg.data_B[j] = msg


    # ***************** check the counter for all target  *****************

    def check_message_counter(self):

        """ This function used to cehck all counters are the same before
        publishing the message to the topic"""
        # -- check for that all message have the same counter
        # -- which mean all message are from the current cycle
        for i in range(len(self.global_msg.data_A)):
            if ((self.global_msg.data_A[0].counter != self.global_msg.data_A[i].counter) or \
              (self.global_msg.data_A[0].counter != self.global_msg.data_B[i].counter)):
                rospy.loginfo('counter is not equal')
                return False

        return True

#  ****************************************************************************

if __name__ == '__main__':
    try:
        RADAR_NODE = RadarPaNode()

        # -- spin simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




