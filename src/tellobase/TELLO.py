#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image , Imu
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class TELLO:
    def __init__(self, tello_id):
        self.rate = rospy.Rate(60)
        self.tello_id = tello_id
        self.odometry = Odometry()
        self.imu = Imu()
        self.image = Image()
        self.empty = Empty()
        self.vel = Twist()
        
        ############### Publishers ##############
        self.vel_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 20)
        self.takeoff_pub = rospy.Publisher("/tello/takeoff", Empty, queue_size=10)
        self.land_pub = rospy.Publisher("/tello/land", Empty, queue_size=10)
        self.fastmode_pub = rospy.Publisher("/tello/fast_mode",Empty,queue_size=10)

    
        ########## Subscribers ##################
        self.odometry_sub = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        self.imu_sub = rospy.Subscriber("/tello/imu", Imu, self.imu_callback, queue_size=10)
        self.image_sub = rospy.Subscriber("/tello/image_raw", Image, self.image_callback)        
        

        ###### Callback Functions ##########
    def odometry_callback(self, odometry_data):
        self.odometry = odometry_data


    def imu_callback(self, imu_data):
        self.imu = imu_data
    
    def image_callback(self, image_data):
        self.image = image_data
 
    def takeoff(self):
        for i in range(60):
            self.takeoff_pub.publish()
            self.rate.sleep()
    
    def land(self):
        self.land_pub.publish()
    
    def set_velocity(self,x,y,z,ax=0,ay=0,az=0):
        # x positivo tello vai pra frente
        # y positivo tello vai pra esquerda
        self.vel.linear.x = x
        self.vel.linear.y = y
        self.vel.linear.z = z
        self.vel.angular.x = ax
        self.vel.angular.y = ay
        self.vel.angular.z = az
        self.vel_pub.publish(self.vel)


if __name__ == '__main__':
    rospy.init_node('tello_test')
    tello = TELLO("Robin")
    tello.takeoff()
    for i in range(60):
        rospy.loginfo("Indo pra frente")
        tello.set_velocity(0.5,0,0)
        tello.rate.sleep()
    for i in range(60):
        rospy.loginfo("Indo pra direita")
        tello.set_velocity(0,-0.5,0)
        tello.rate.sleep()
    for i in range(60):
        rospy.loginfo("Indo pra traz")
        tello.set_velocity(-0.5,0,0)
        tello.rate.sleep()
    for i in range(60):
        rospy.loginfo("Indo pra esquerda")
        tello.set_velocity(0,0.5,0)
        tello.rate.sleep()
    for i in range(60):
        rospy.loginfo("Parando")
        tello.set_velocity(0,0,0)
        tello.rate.sleep()
    tello.land()
