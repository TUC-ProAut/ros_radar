#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *****************************************************************************
#                                                                             *
# radar2pcd_pa_node.py                                                        *
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
# This node converts the radar_messages to PointCloud.msg                     *
#                                                                             *
# Supported radars:                                                           *
#     * Bosch GPR v1.0                                                        *
#                                                                             *
# *****************************************************************************

# --  standard headers
import math

# --  ros reahder
import rospy
import std_msgs.msg
from radar_pa_msgs.msg   import   radar_msg
from sensor_msgs.msg     import   PointCloud
from sensor_msgs.msg     import   ChannelFloat32
from geometry_msgs.msg   import   Point32

class radar2pcdPaNode():

    def __init__(self):

        rospy.init_node('radar2pcd_pa_node.py', anonymous=True)
        rospy.Subscriber('radar_messages', radar_msg, self.callback)

        self.pcpub = rospy.Publisher('radar_pcd', PointCloud, queue_size=10)

        #  PointCloud
        self.out_Pc = PointCloud()
        self.out_Pc.header = std_msgs.msg.Header()
        self.out_Pc.header.stamp = rospy.Time.now()
        self.out_Pc.header.frame_id = "radar"

        num_targets = 48
        # -- None here represent data type # this data type is point
        self.out_Pc.points = [None] * num_targets

        # -- channels must be ChannelFloat32()
        self.out_Pc.channels = [ChannelFloat32(), ChannelFloat32(), \
          ChannelFloat32(), ChannelFloat32(), ChannelFloat32(), \
          ChannelFloat32(), ChannelFloat32(), ChannelFloat32(), \
          ChannelFloat32(), ChannelFloat32()]

        self.out_Pc.channels[0].values = [None] * num_targets
        self.out_Pc.channels[1].values = [None] * num_targets
        self.out_Pc.channels[2].values = [None] * num_targets
        self.out_Pc.channels[3].values = [None] * num_targets
        self.out_Pc.channels[4].values = [None] * num_targets
        self.out_Pc.channels[5].values = [None] * num_targets
        self.out_Pc.channels[6].values = [None] * num_targets
        self.out_Pc.channels[7].values = [None] * num_targets
        self.out_Pc.channels[8].values = [None] * num_targets
        self.out_Pc.channels[9].values = [None] * num_targets

        self.out_Pc.channels[0].name = 'ID'
        self.out_Pc.channels[1].name = 'distance'
        self.out_Pc.channels[2].name = 'velocity'
        self.out_Pc.channels[3].name = 'power'
        self.out_Pc.channels[4].name = 'angle'
        self.out_Pc.channels[5].name = 'distance_deviation'
        self.out_Pc.channels[6].name = 'angle_deviation'
        self.out_Pc.channels[7].name = 'velocity_deviation'
        self.out_Pc.channels[8].name = 'proability_target'
        self.out_Pc.channels[9].name = 'is_target'

    def callback(self, data):
        self.out_Pc.header.stamp = data.header.stamp

        for i in range(len(self.out_Pc.points)):


            # -- check if target is valid
            if (data.data_A[i].is_target == 1):

                # -- compute each x,y for each point
                x = data.data_A[i].distance * (math.cos(data.data_A[i].angle))
                y = data.data_A[i].distance * (math.sin(data.data_A[i].angle))

                # --  fill the channels wit Id , distance,..
                self.out_Pc.channels[0].values[i] = data.data_A[i].ID
                self.out_Pc.channels[1].values[i] = data.data_A[i].distance
                self.out_Pc.channels[2].values[i] = data.data_A[i].velocity
                self.out_Pc.channels[3].values[i] = data.data_A[i].power
                self.out_Pc.channels[4].values[i] = data.data_A[i].angle
                self.out_Pc.channels[5].values[i] = data.data_B[i].distance_deviation
                self.out_Pc.channels[6].values[i] = data.data_B[i].angle_deviation
                self.out_Pc.channels[7].values[i] = data.data_B[i].velocity_deviation
                self.out_Pc.channels[8].values[i] = data.data_B[i].proability_target
                self.out_Pc.channels[9].values[i] = data.data_A[i].is_target

            else:
                # -- assign NAN to not valid target
                x = float('nan')
                y = float('nan')
                self.out_Pc.channels[0].values[i] = float('nan')
                self.out_Pc.channels[1].values[i] = float('nan')
                self.out_Pc.channels[2].values[i] = float('nan')
                self.out_Pc.channels[3].values[i] = float('nan')
                self.out_Pc.channels[4].values[i] = float('nan')
                self.out_Pc.channels[5].values[i] = float('nan')
                self.out_Pc.channels[6].values[i] = float('nan')
                self.out_Pc.channels[7].values[i] = float('nan')
                self.out_Pc.channels[8].values[i] = float('nan')
                self.out_Pc.channels[9].values[i] = float('nan')

            self.out_Pc.points[i] = Point32(x, y, 0)

        self.pcpub.publish(self.out_Pc)

#  ****************************************************************************

if __name__ == '__main__':

    try:
        radar2pcdNode = radar2pcdPaNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

