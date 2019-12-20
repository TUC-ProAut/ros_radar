#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *****************************************************************************
#                                                                             *
# extract_message.py                                                          *
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
# This file converts the given data from the can messages to radar_msgs.      *
#                                                                             *
# Supported radars:                                                           *
#     * Bosch GPR v1.0                                                        *
#                                                                             *
# *****************************************************************************

# -- standard modules
import sys

# -- ros modules
import rospy
from radar_pa_msgs.msg import radar_msg_A
from radar_pa_msgs.msg import radar_msg_B

def extract_GPR_v10(received_data): # (): # -- to test the function

    # -- use for test the function , function argument must removed
    #received_data = [128, 255,7, 128, 85, 0, 192, 244] # only for test
    # -- received_data have 8-bit string so using ord return integer
    data = data_ord(received_data)

    # -- depend on the Amessage to know if it A or B message S
    if ((data[0] & 0b00000001) == 0b00000001):
        # -- message A
        msg = radar_msg_A()
        msg.message            = extract_bit (0,  1, data)
        msg.ID                 = extract_bit (1,  6, data)
        msg.distance           = (extract_bit(7, 12, data)) * 0.0625                        # unit m
        msg.velocity           = (twos_complement( extract_bit(19, 12, data), 12)) * 0.0625   # unit m/s
        msg.power              = (twos_complement( extract_bit(31,  8, data),  8)) * 0.5         # unit dBm2
        msg.angle              = (twos_complement( extract_bit(39, 14, data), 14)) * 0.0002  # rad
        msg.is_target          = extract_bit(53,  1, data)
        msg.counter            = extract_bit(54,  2, data)

    else:
        # -- message B
        msg                    = radar_msg_B()
        msg.message            = extract_bit( 0,  1, data)
        msg.ID                 = extract_bit( 1,  6, data)
        msg.angle_deviation    = extract_bit( 7,  6, data)    * 0.001   # rad
        msg.velocity_deviation = extract_bit(13,  6, data)    * 0.0625  # m/s
        msg.distance_deviation = extract_bit(19,  6, data)    * 0.0625  # m
        msg.proability_target  = extract_bit(25,  5, data)    * 0.03125
        #msg.time              = extract_bit(30, 13, data)    * 0.0001  # s
        msg.counter            = extract_bit(54,  2, data)

    return msg

# ***************** extract_bit *****************

def extract_bit(start, length, data):

    # -- get the byte number
    byteNum = start / 8
    #-- get the bit number
    bitNum = start % 8

    if  ((bitNum + length) <= 8):
        fisrt_byte = data[byteNum] >>(bitNum) & mask_value(length)
        new_data = fisrt_byte

    elif ((bitNum + length) <= 16):
        fisrt_byte = data[byteNum] >>(bitNum) & mask_value(8-bitNum)
        second_byte = data[byteNum+1] <<(8-bitNum) & mask_value(length)
        new_data = fisrt_byte | second_byte

    elif ((bitNum + length) <= 24):
        fisrt_byte = data[byteNum] >>(bitNum) & mask_value(8-bitNum)
        second_byte = data[byteNum+1] <<(8-bitNum) & mask_value(8+bitNum)
        third_byte = data[byteNum+2] <<(8+(8-bitNum)) & mask_value(length)
        new_data = fisrt_byte | second_byte |third_byte

    else:
        print ('out of range')

    return(new_data)

# ***************** generate the maske value *****************

def mask_value(length):

    # -- we can do this by using two methods
    # -- method 1 with multipliaction or shift
    # -- we can get the data by make it 2 ** (length+1 ) - 1
    #print ('the value of mask is ',bin(((1<<length))-1))
    # -- (2**(length) == (1<<length)
    #return ((2**(length))-1)

    return ((1 << length)-1)

    # method 2   with for loop
    #sum_1 = 0
    #for i in range (0 ,x):
        #sum_1 = sum_1 + 2**i
    #print ('totl sum', sum_1 , bin(sum_1))
    #return (sum_1)

# ***************** convert the message from str to int *****************

def data_ord(data):

    #result = [0] * len(data)
    result = []
    for i in range(len(data)):
        if ((type(data[i]) is str)):
            #result[i] = ord (data[i])
            result.append(ord(data[i]))
        else:
            #result[i] = data[i]
            result.append(data[i])

    return (result)

#***************** two's complement function *****************

def twos_complement(input_data, length):

    if (input_data & (1<<(length-1))):
        input_data -= (1<<length)

    return (input_data)

#***************** count number of bit *****************

def bit_len(input_data):

    data_length = 0
    while (input_data):
        input_data >>= 1
        data_length += 1

    return (data_length)
