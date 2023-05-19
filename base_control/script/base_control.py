#!/usr/bin/python3.8
# coding=utf8

# Copyright 2019 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import signal
import os
import rospy
import tf
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range

import ctypes
import numpy as np

base_type = os.getenv('BASE_TYPE','NanoRobot')
if os.getenv('SONAR_NUM') is None:
    sonar_num = 0
else:
    sonar_num = int(os.getenv('SONAR_NUM')) 


#class queue is design for uart receive data cache
class queue:
    def __init__(self, capacity = 1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        for i in range(self.capacity):
            pass
            print(self.array[i])
        print(' ')

#class BaseControl is design for hardware base relative control
class BaseControl:
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.device_port = rospy.get_param('~port','/dev/ttyUSB0')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','50'))
        self.odom_topic = rospy.get_param('~odom_topic','/odom')
        self.battery_topic = rospy.get_param('~battery_topic','battery')
        self.battery_freq = float(rospy.get_param('~battery_freq','1'))
        self.cmd_vel_topic= rospy.get_param('~cmd_vel_topic','/cmd_vel')
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic','/ackermann_cmd_topic')

        self.pub_imu = bool(rospy.get_param('~pub_imu',False))
        if(self.pub_imu == True):
            self.imuId = rospy.get_param('~imu_id','imu')
            self.imu_topic = rospy.get_param('~imu_topic','imu')
            self.imu_freq = float(rospy.get_param('~imu_freq','50'))
            if self.imu_freq > 100:
                self.imu_freq = 100

        self.pub_sonar = bool(rospy.get_param('~pub_sonar',False))
        self.sub_ackermann = bool(rospy.get_param('~sub_ackermann',False))


        #define param
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
       
        # bingo start
        self.init_encode_a = False
        self.current_encode_a = 0
        self.previous_encode_a = 0

        self.init_encode_b = False
        self.current_encode_b = 0
        self.previous_encode_b = 0

        self.sum_encode_a = 0
        self.sum_encode_b = 0
        self.odom_x = 0
        self.odom_y = 0
        self.yaw_angle = 0

        self.diff_encode_a = 0
        self.diff_encode_b = 0
        self.tf_cnt = 0
        # bingo end

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.serialIDLE_flag = 0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.sendcounter = 0
        self.ImuErrFlag = False
        self.EncoderFlag = False
        self.BatteryFlag = False
        self.OdomTimeCounter = 0
        self.BatteryTimeCounter = 0
        self.Circleloop = queue(capacity = 1024*4)
        self.Vx = 0
        self.Vy = 0
        self.Vyaw = 0
        self.Yawz = 0
        self.Vvoltage = 0
        self.Icurrent = 0
        self.Gyro = [0,0,0]
        self.Accel = [0,0,0]
        self.Quat = [0,0,0,0]
        self.Sonar = [0,0,0,0]
        self.movebase_firmware_version = [0,0,0]
        self.movebase_hardware_version = [0,0,0]
        self.movebase_type = ["NanoCar","NanoRobot","4WD_OMNI","4WD","RC_ACKERMAN"]
        self.motor_type = ["25GA370","37GB520","TT48","RS365","RS540"]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_ackermann_cmd_time = rospy.Time.now()
        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=10)
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.loginfo("Opening Serial Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")
        #if move base type is ackermann car like robot and use ackermann msg ,sud ackermann topic,else sub cmd_vel topic
        if(('NanoCar' in base_type) & (self.sub_ackermann == True)):
            from ackermann_msgs.msg import AckermannDriveStamped
            self.sub = rospy.Subscriber(self.ackermann_cmd_topic,AckermannDriveStamped,self.ackermannCmdCB,queue_size=20)
        else:
            self.sub = rospy.Subscriber(self.cmd_vel_topic,Twist,self.cmdCB,queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10)
        self.battery_pub = rospy.Publisher(self.battery_topic,BatteryState,queue_size=3)
        if self.pub_sonar:
            if sonar_num > 0:
                self.range_pub1 = rospy.Publisher('sonar_1',Range,queue_size=3)
            if sonar_num > 1:    
                self.range_pub2 = rospy.Publisher('sonar_2',Range,queue_size=3)
            if sonar_num > 2:
                self.range_pub3 = rospy.Publisher('sonar_3',Range,queue_size=3)
            if sonar_num > 3:
                self.range_pub4 = rospy.Publisher('sonar_4',Range,queue_size=3)
            if sonar_num > 0:
                self.timer_sonar = rospy.Timer(rospy.Duration(100.0/1000),self.timerSonarCB)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/50),self.timerOdomCB2)
        # self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq),self.timerBatteryCB)  
        self.timer_communication = rospy.Timer(rospy.Duration(1.0/200),self.timerCommunicationCB)

        #inorder to compatibility old version firmware,imu topic is NOT pud in default
        if(self.pub_imu):
            self.imu_pub = rospy.Publisher(self.imu_topic,Imu,queue_size=10)
            self.timer_imu = rospy.Timer(rospy.Duration(1.0/self.imu_freq),self.timerIMUCB) 
        # self.getVersion()
        # #move base imu initialization need about 2s,during initialization,move base system is blocked
        # #so need this gap
        # while self.movebase_hardware_version[0] == 0:
        #     pass
        # # if self.movebase_hardware_version[0] < 2:
        # #     print self.movebase_hardware_version
        # #     time.sleep(2.0)
        # self.getSN()
        # time.sleep(0.01)
        # self.getInfo()
        self.openCanAT()
    
    #CRC-8 Calculate
    def crc_1byte(self,data):
        crc_1byte = 0
        for i in range(0,8):
            if((crc_1byte^data)&0x01):
                crc_1byte^=0x18
                crc_1byte>>=1
                crc_1byte|=0x80
            else:
                crc_1byte>>=1
            data>>=1
        return crc_1byte
    def crc_byte(self,data,length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret^data[i])
        return ret               
    
    def rpm_to_velocity(self, encode):
        '''
        rpm 值 为0.001rpm
        '''
        R = 0.085
        # rpm = encode * 1875 / (512 * 4096)
        rpm = encode * 0.001
        velocity = (rpm / 60) * math.pi * R
        return velocity

    #Subscribe vel_cmd call this to send vel cmd to move base
    def cmdCB(self,data):
        self.trans_x = data.linear.x
        self.trans_y = data.linear.y
        self.rotat_z = data.angular.z
        self.last_cmd_vel_time = rospy.Time.now()
        
        rospy.loginfo("%f,%f,%f",data.linear.x,data.linear.y,data.angular.z)
        R = 0.085
        round = 2 * math.pi * R

        diff = data.angular.z * R
       
        
        Head_B = [0x41,0x54,0xC0,0x40,0x00,0x00]
        Head_A = [0x41,0x54,0xC0,0x20,0x00,0x00]
        Tail = [0x0D,0x0A]
        
        # 转圈速转速度
        # velocity = (rpm / 60) * math.pi * R
        
        
        # encode = 24
        # rpm = encode * 1875 / (512 * 4096)
        # rpm = encode * 0.001
        # velocity = (rpm / 60) * math.pi * R
        # print(velocity)


        rpm = ((data.linear.x + diff ) / (math.pi* R)) * 60
        rpm_d = 0 - ((data.linear.x - diff )/ (math.pi * R)) * 60

        
        d = int((rpm_d * 512* 4096) / 1875)
        # print('d',d)
        spd_a0 = (d >> 24) & 0xff
        spd_a1 = (d >> 16) & 0xff
        spd_a2 = (d >> 8) & 0xff
        spd_a3 = d & 0xff

        d = int((rpm * 512 * 4096) / 1875)
        spd_b0 = (d >> 24) & 0xff
        spd_b1 = (d >> 16) & 0xff
        spd_b2 = (d >> 8) & 0xff
        spd_b3 = d & 0xff
        
        
        # print(sp1)
                                                                                                                 #00
        # speed_cmd = [0x08,0x23,0xFF,0x60,0x00,0x5C,0x8F,0x02,0x00]
        speed_cmd_a = [0x08,0x23,0xFF,0x60,0x00,spd_a3,spd_a2,spd_a1,spd_a0]
        speed_cmd_b = [0x08,0x23,0xFF,0x60,0x00,spd_b3,spd_b2,spd_b1,spd_b0]
        
        enable_cmd =  [0x08,0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00]
        disable_cmd = [0x08,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00]
        
        en_b = disable_cmd
        en_a = disable_cmd

        if  data.linear.x != 0 or data.angular.z !=0:
            en_a = enable_cmd
            en_b = enable_cmd
        else:
            pass
            en_a = disable_cmd
            en_b = disable_cmd



        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            
           
            # print(Head_A + en_a + Tail)
            self.serial.write(Head_A + en_a + Tail)
            
            self.serial.write(Head_A + speed_cmd_a + Tail)
            # time.sleep(0.1)
            self.serial.write(Head_B + en_b + Tail)
            self.serial.write(Head_B + speed_cmd_b + Tail)
            
            
            
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0
    
    #Subscribe ackermann Cmd call this to send vel cmd to move base
    def ackermannCmdCB(self,data):
        self.speed = data.drive.speed
        self.steering_angle = data.drive.steering_angle
        self.last_ackermann_cmd_time = rospy.Time.now()
        output = chr(0x5a) + chr(12) + chr(0x01) + chr(0x15) + \
            chr((int(self.speed*1000.0)>>8)&0xff) + chr(int(self.speed*1000.0)&0xff) + \
            chr(0x00) + chr(0x00) + \
            chr((int(self.steering_angle*1000.0)>>8)&0xff) + chr(int(self.steering_angle*1000.0)&0xff) + \
            chr(0x00)
        outputdata = [0x5a,0x0c,0x01,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(self.speed*1000.0)>>8)&0xff
        outputdata[5] = int(self.speed*1000.0)&0xff
        outputdata[8] = (int(self.steering_angle*1000.0)>>8)&0xff
        outputdata[9] = int(self.steering_angle*1000.0)&0xff
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0


    #Communication Timer callback to handle receive data
    #depend on communication protocol
    def timerCommunicationCB(self,event):
        length = self.serial.in_waiting

        if length:            
            reading = self.serial.read_all()
            # print('len', len(reading))
            if len(reading)!=0:
                for i in range(0,len(reading)):
                    # data = (int(reading[i].encode('utf-8'),16)) 
                    # data = reading[i].encode('utf-8')
                    data = reading[i]
                    # print('%#x'%data,end=' ')
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        pass
            # print()
        else:
            pass

        # l = self.Circleloop.get_queue_length()
        # print('d',l)
        # self.current_time = rospy.Time.now()
        # dt = (self.current_time - self.previous_time).to_sec()
        # self.previous_time = self.current_time

       
        if self.Circleloop.get_queue_length() >= 60:  
            data = self.Circleloop.get_front() 
            if 0x41 == data:
                self.Circleloop.dequeue()
                if 0x54 == self.Circleloop.get_front():
                    # print('AT-')
                    self.Circleloop.dequeue()
                    databuf = []
                    if self.Circleloop.get_queue_length() > 15:
                        for i in range(15):
                            d = self.Circleloop.get_front()
                            databuf.append(d)
                            self.Circleloop.dequeue()
                        #     print('%#x'%d, end=' ')
                        # print('end=',databuf[13],databuf[14])
                    
                    if databuf[1] == 0x20 and databuf[0] == 0x30:
                        
                        rpm = np.int32(((databuf[8]&0xff)<<24)|((databuf[7]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[5]&0xff))
                        velocity = self.rpm_to_velocity(rpm)
                        # print('v1=',rpm)
                       
                        
                        encode = np.int32(((databuf[12]&0xff)<<24)|((databuf[11]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[9]&0xff))
                        
                        self.current_encode_a = encode
                        if self.init_encode_a == True:
                            self.diff_encode_a = (self.previous_encode_a - self.current_encode_a)
                            self.previous_encode_a = self.current_encode_a
                            if -200 < self.diff_encode_a < 200:
                                self.sum_encode_a += self.diff_encode_a
                            else:
                                print('diff=',self.diff_encode_a)
                        else:
                            self.previous_encode_a = self.current_encode_a
                            self.init_encode_a = True
                        # print('encode_a=', diff_encode,self.sum_encode_a)
                    elif databuf[1] == 0x40 and databuf[0] == 0x30:
                        rpm = np.int32(((databuf[8]&0xff)<<24)|((databuf[7]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[5]&0xff))
                        velocity = self.rpm_to_velocity(rpm)
                        # print('v2=',rpm)

                        encode = np.int32(((databuf[12]&0xff)<<24)|((databuf[11]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[9]&0xff))

                        self.current_encode_b = encode

                        if self.init_encode_b == True:
                            self.diff_encode_b = (self.current_encode_b - self.previous_encode_b)
                            self.previous_encode_b = self.current_encode_b
                            if -200 < self.diff_encode_b < 200:
                                self.sum_encode_b += self.diff_encode_b

                            angle = ((self.sum_encode_b - self.sum_encode_a) / 4096) * 180 * 0.17 / 0.38
                            angle = angle % 360
                            # rad = rad * 180 / math.pi
                            self.yaw_angle = angle
                            d = ((self.diff_encode_a + self.diff_encode_b)/2/4096)*math.pi*0.17
                            rad = angle * math.pi /180 
                            
                            self.odom_x += d*math.cos(rad)
                            self.odom_y += d*math.sin(rad)
                            # print('encode_b=', self.diff_encode_b ,'==',self.sum_encode_b, '--' , self.sum_encode_a, 'ODOM', 'X=',round(self.odom_x,2) , 'Y=',round(self.odom_y,2),'angle=',round(angle,2))
                        else:
                            self.previous_encode_b = self.current_encode_b
                            self.init_encode_b = True
                    else:
                         print("data error")
                else:
                    self.Circleloop.dequeue()
            
            else:
                print('ext %#x'%data)
                self.Circleloop.dequeue()


        
    #get move base hardware & firmware version    

    def getVersion(self):
        #Get version info
        
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf1) + chr(0x00) + chr(0xd7) #0xd7 is CRC-8 value
        output = output.encode("utf-8")
        
        while(self.serialIDLE_flag):
            
            time.sleep(0.01)
        
        self.serialIDLE_flag = 1
        
        try:
            while self.serial.out_waiting:
                pass
           
            self.serial.write(output)

        except:
            rospy.logerr("Get Version Command Send Faild~")
        self.serialIDLE_flag = 0
        
    #get move base SN
    def getSN(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf3) + chr(0x00) + chr(0x46) #0x46 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get SN Command Send Faild")
        self.serialIDLE_flag = 0

    #get move base info
    def getInfo(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x21) + chr(0x00) + chr(0x8f) #0x8f is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get info Command Send Faild")
        self.serialIDLE_flag = 0   

    def openCanAT(self):
        #Get version info
        AT_CMD = [0x41,0x54, 0x2B, 0x41,0x54,0x0D,0x0A]
        
        Head_B = [0x41,0x54,0xC0,0x40,0x00,0x00]
        Head_A = [0x41,0x54,0xC0,0x20,0x00,0x00]
        Tail = [0x0D,0x0A]
        PDO = [0x08, 0x2F, 0x00, 0x47, 0x01, 0x01, 0x00, 0x00, 0x00]
        PDO_inervalTime = [0x08, 0x2B, 0x00, 0x18, 0x03, 0x14, 0x00, 0x00, 0x00]

        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            
            self.serial.write(AT_CMD)
            self.serial.write(AT_CMD)
            time.sleep(0.5)
            self.serial.write(Head_A + PDO + Tail)
            time.sleep(0.5)
            self.serial.write(Head_A + PDO_inervalTime + Tail)
            time.sleep(0.5)
            self.serial.write(Head_B + PDO + Tail)
            time.sleep(0.5)
            self.serial.write(Head_B + PDO_inervalTime + Tail)
        except:
            rospy.logerr("Open can AT MODE Send Faild")
        self.serialIDLE_flag = 0


    #Odom Timer call this to get velocity and imu info and convert to odom topic
    def timerOdomCB(self,event):
        #Get move base velocity data
        if self.movebase_firmware_version[1] == 0: 
            #old version firmware have no version info and not support new command below
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x09) + chr(0x00) + chr(0x38) #0x38 is CRC-8 value
        else:
            #in firmware version new than v1.1.0,support this command      
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x11) + chr(0x00) + chr(0xa2) 
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Odom Command Send Faild")
        self.serialIDLE_flag = 0   
        #calculate odom data
        Vx = float(ctypes.c_int16(self.Vx).value/1000.0)
        Vy = float(ctypes.c_int16(self.Vy).value/1000.0)
        Vyaw = float(ctypes.c_int16(self.Vyaw).value/1000.0)

        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value/100.0)
        self.pose_yaw = self.pose_yaw*math.pi/180.0
  
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
        self.previous_time = self.current_time
        self.pose_x = self.pose_x + Vx * (math.cos(self.pose_yaw))*dt - Vy * (math.sin(self.pose_yaw))*dt
        self.pose_y = self.pose_y + Vx * (math.sin(self.pose_yaw))*dt + Vy * (math.cos(self.pose_yaw))*dt

        pose_quat = tf.transformations.quaternion_from_euler(0,0,self.pose_yaw)        
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id =self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = Vx
        msg.twist.twist.linear.y = Vy
        msg.twist.twist.angular.z = Vyaw
        self.pub.publish(msg)
        self.tf_broadcaster.sendTransform((self.pose_x,self.pose_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
    
    def timerOdomCB2(self,event):
        #Get move base velocity data
        
        self.current_time = rospy.Time.now()
        self.tf_cnt += 1
        if self.tf_cnt % 50 == 0:
            
            print('tf',self.odom_x,self.odom_y)
        pose_quat_zero = tf.transformations.quaternion_from_euler(0,0,0) 
        pose_quat = tf.transformations.quaternion_from_euler(0,0,self.yaw_angle*math.pi /180.0)        
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id =self.baseId
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = 0.1
        msg.twist.twist.linear.y = 0.1
        msg.twist.twist.angular.z = 0.05
        self.pub.publish(msg)
        self.tf_broadcaster.sendTransform((self.odom_x,self.odom_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
        self.tf_broadcaster.sendTransform((0.0,0.0,0.0),pose_quat_zero,self.current_time,'base_link',self.baseId)
        self.tf_broadcaster.sendTransform((0.24,0.0,0.0),pose_quat_zero,self.current_time,'laser_link','base_link')

    
    #Battery Timer callback function to get battery info
    def timerBatteryCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x07) + chr(0x00) + chr(0xe4) #0xe4 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Battery Command Send Faild")
        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage/1000.0)
        msg.current = float(self.Icurrent/1000.0)
        self.battery_pub.publish(msg)
    
    #Sonar Timer callback function to get battery info
    def timerSonarCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x19) + chr(0x00) + chr(0xff) #0xff is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Sonar Command Send Faild")
        self.serialIDLE_flag = 0
        msg = Range()
        msg.header.stamp = self.current_time
        msg.field_of_view = 0.26 #about 15 degree
        msg.max_range = 2.5
        msg.min_range = 0.01
        # Sonar 1
        msg.header.frame_id = 'Sonar_1'
        if self.Sonar[0] == 0xff:
            msg.range = float('inf') 
        else:
            msg.range = self.Sonar[0] / 100.0
        self.range_pub1.publish(msg)
         
        # TF value calculate from mechanical structure
        if('NanoRobot' in base_type ):
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0 ),(0.0, 0.0, 0.0, 1.0),self.current_time,'Sonar_1',self.baseId)
        elif('NanoCar' in base_type):
            self.tf_broadcaster.sendTransform((0.18, 0.0, 0.0 ),(0.0, 0.0, 0.0, 1.0),self.current_time,'Sonar_1',self.baseId)
        elif('4WD' in base_type):
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0 ),(0.0, 0.0, 0.0, 1.0),self.current_time,'Sonar_1',self.baseId)
        elif('Race182' in base_type):
            self.tf_broadcaster.sendTransform((0.18, 0.0, 0.0 ),(0.0, 0.0, 0.0, 1.0),self.current_time,'Sonar_1',self.baseId)   
        elif('NanoOmni' in base_type):
            self.tf_broadcaster.sendTransform((0.11, 0.0, 0.0 ),(0.0, 0.0, 0.0, 1.0),self.current_time,'Sonar_1',self.baseId)              
        else:
            pass

        # Sonar 2
        if sonar_num > 1:   
            if self.Sonar[1] == 0xff:
                msg.range = float('inf') 
            else:
                msg.range = self.Sonar[1] / 100.0
            msg.header.frame_id = 'Sonar_2'
            self.range_pub2.publish(msg)
            
            if('NanoRobot' in base_type):
                self.tf_broadcaster.sendTransform((0.0, 0.0 ,0.0 ),(0.0,0.0,-1.0,0),self.current_time,'Sonar_2',self.baseId) 
            elif('NanoCar' in base_type):
                self.tf_broadcaster.sendTransform((-0.035, 0.0 ,0.0 ),(0.0,0.0,-1.0,0),self.current_time,'Sonar_2',self.baseId) 
            elif('4WD' in base_type):
                self.tf_broadcaster.sendTransform((0.0, 0.0 ,0.0 ),(0.0,0.0,-1.0,0),self.current_time,'Sonar_2',self.baseId) 
            elif('Race182' in base_type):
                self.tf_broadcaster.sendTransform((-0.08, 0.0 ,0.0 ),(0.0,0.0,-1.0,0),self.current_time,'Sonar_2',self.baseId)     
            elif('NanoOmni' in base_type):
                self.tf_broadcaster.sendTransform((-0.11, 0.0, 0.0 ),(0.0, 0.0, -1.0, 0.0),self.current_time,'Sonar_2',self.baseId)  
            else:
                pass

        if sonar_num > 2:   
        # Sonar 3
            msg.header.frame_id = 'Sonar_3'
            if self.Sonar[2] == 0xff:
                msg.range = float('inf') 
            else:
                msg.range = self.Sonar[2] / 100.0
            self.range_pub3.publish(msg)
            if('Race182' in base_type):
                self.tf_broadcaster.sendTransform((0.0, 0.06 ,0.0 ),(0.0,0.0,0.707,0.707),self.current_time,'Sonar_3',self.baseId) 
            elif('NanoOmni' in base_type):   
                self.tf_broadcaster.sendTransform((0.0, 0.07 ,0.0 ),(0.0,0.0,0.707,0.707),self.current_time,'Sonar_3',self.baseId)          
        if sonar_num > 3:   
        # Sonar 4
            if self.Sonar[3] == 0xff:
                msg.range = float('inf') 
            else:
                msg.range = self.Sonar[3] / 100.0
            msg.header.frame_id = 'Sonar_4'
            self.range_pub4.publish(msg)
            if('Race182' in base_type):
                self.tf_broadcaster.sendTransform((0.0, -0.06 ,0.0 ),(0.0,0.0,-0.707,0.707),self.current_time,'Sonar_4',self.baseId) 
            elif('NanoOmni' in base_type):
                self.tf_broadcaster.sendTransform((0.0, -0.07 ,0.0 ),(0.0,0.0,-0.707,0.707),self.current_time,'Sonar_4',self.baseId) 
    
    #IMU Timer callback function to get raw imu info
    def timerIMUCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x13) + chr(0x00) + chr(0x33) #0x33 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Imu Command Send Faild")

        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.imuId

        msg.angular_velocity.x = float(ctypes.c_int32(self.Gyro[0]).value/100000.0)
        msg.angular_velocity.y = float(ctypes.c_int32(self.Gyro[1]).value/100000.0)
        msg.angular_velocity.z = float(ctypes.c_int32(self.Gyro[2]).value/100000.0)

        msg.linear_acceleration.x = float(ctypes.c_int32(self.Accel[0]).value/100000.0)
        msg.linear_acceleration.y = float(ctypes.c_int32(self.Accel[1]).value/100000.0)
        msg.linear_acceleration.z = float(ctypes.c_int32(self.Accel[2]).value/100000.0)

        msg.orientation.w = float(ctypes.c_int16(self.Quat[0]).value/10000.0)
        msg.orientation.x = float(ctypes.c_int16(self.Quat[1]).value/10000.0)
        msg.orientation.y = float(ctypes.c_int16(self.Quat[2]).value/10000.0)
        msg.orientation.z = float(ctypes.c_int16(self.Quat[3]).value/10000.0)

        self.imu_pub.publish(msg)  
        # TF value calculate from mechanical structure
        if('NanoRobot' in base_type):
            self.tf_broadcaster.sendTransform((-0.062,-0.007,0.08),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId)   
        elif('NanoCar' in base_type):
            self.tf_broadcaster.sendTransform((0.0,0.0,0.09),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId) 
        elif('4WD' in base_type):
            self.tf_broadcaster.sendTransform((-0.065,0.0167,0.02),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId)     
        else:
            self.tf_broadcaster.sendTransform((0.0,0.0,0.0),(0.0,0.0,0.0,1.0),self.current_time,self.imuId,self.baseId)



#main function
if __name__=="__main__":
    try:
        rospy.init_node('base_control',anonymous=True)
        if base_type != None:
            rospy.loginfo('%s base control ...'%base_type)
        else:
            rospy.loginfo('base control ...')
            rospy.logerr('PLEASE SET BASE_TYPE ENV FIRST')

        bc = BaseControl()
        def sigint_handler(signum, frame):
            bc.serial.close
            return

            

        signal.signal(signal.SIGINT, sigint_handler)

        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
