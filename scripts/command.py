#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros.msg import Detection
from darknet_ros.srv import GetDetections, GetDetectionsResponse
from std_srvs.srv import Empty

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import time

import cv2
import os
import numpy as np
import darknet_cv2 as darknet

class DarknetROS():

    def __init__(self):

        rospy.init_node('darknet_ros')
        
        rgb_topic = "/Realsense_Camera/RGB/image_raw"
        # rgb_topic = "/hsrb/head_rgbd_sensor/rgb/image_raw"
        # rgb_topic = "/hsrb/head_r_stereo_camera/image_raw"

        
              
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_cb)
        # self.detection_srv = rospy.Service('/darknet_ros/detect', Empty, self.detection_service)
        self.detection_srv = rospy.Service('/darknet_ros/detect', GetDetections, self.detection_service)

        self.cv_bridge = CvBridge()


        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('darknet_ros')
        cfg_dir = pkg_dir + "/cfg/"
        weights_dir = pkg_dir + "/weights/"

        self.groceries_network = darknet.load_net(cfg_dir+"yolov3-tiny-sign.cfg", weights_dir+"yolov3-tiny-sign_final.weights", 0)
        self.groceries_meta = darknet.load_meta(cfg_dir+"sign.data")
        rospy.sleep(1) # settle down

    def rgb_cb(self, msg):

        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.first_image = True

        except CvBridgeError, e:
            a=e

    def detection_service(self, req):
        PI = 3.1415926535897
        velocity_publisher = rospy.Publisher('/hamer/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        
        
        while (1):
            img = self.frame
            results = darknet.detect(self.groceries_network, self.groceries_meta, img, thresh=0.35)
            speed=1.05
            print("Ortam taraniyor...!")
            detections = []
            velocity_publisher.publish(vel_msg)
            
            print results
            for box in results:
                label = box[0]
                conf = round(box[1],2)
                w = int(box[2][2])
                h = int(box[2][3])
                cx = int(box[2][0])
                cy = int(box[2][1])
                bx = int(cx-(w/2))
                by = int(cy-(h/2))
                detection = Detection()
                detection.label = label
                detection.bbox = [bx,by,bx+w,by+h]
                detection.accuracy = box[1]
                img = cv2.rectangle(img,(bx,by),
                                    (bx+w,by+h),
                                    (0,0,255),
                                    2)
                cv2.putText(img, label, (bx, by - 12),
                    0, 1e-3 * img.shape[0],(0,0,255),2)
                cv2.putText(img, str(conf), (bx, by - 36),
                    0, 1e-3 * img.shape[0],(0,0,255),2)
                detections.append([detection])
                
                
                
                if(label=='dur'):
                    print("Dur!!")
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                    velocity_publisher.publish(vel_msg)
                    cv2.imwrite('/home/cemal/Desktop/signs/dur.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])
                    break
                elif(label=='sola don'):

                    print("Sola donus!")
                    speed = 40
                    angle = 130
                    clockwise = 0 #True or false
                    time.sleep(5)
                    #Converting from angles to radians
                    angular_speed = speed*2*PI/360
                    relative_angle = angle*2*PI/360

                    #We wont use linear components
                    vel_msg.linear.x=0
                    vel_msg.linear.y=0
                    vel_msg.linear.z=0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0

                    # Checking if our movement is CW or CCW
                    if clockwise:
                        vel_msg.angular.z = -abs(angular_speed)
                    else:
                        vel_msg.angular.z = abs(angular_speed)
                    # Setting the current time for distance calculus
                    
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
                    
                    while(current_angle < relative_angle):
                        velocity_publisher.publish(vel_msg)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed*(t1-t0)
                        
               
                    vel_msg.angular.z = 0    
                

                    velocity_publisher.publish(vel_msg) 

                    cv2.imwrite('/home/cemal/Desktop/signs/sol.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])

                    vel_msg.linear.x = 1
                    velocity_publisher.publish(vel_msg)


                elif(label=='ileri'):
                    print("ileri!")
                    vel_msg.linear.x = abs(speed)
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0         
                    velocity_publisher.publish(vel_msg)
                    cv2.imwrite('/home/cemal/Desktop/signs/ileri.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])
                    
        print detections
        return GetDetectionsResponse(detections)            

        


if __name__ == '__main__':

    dark = DarknetROS()
    dark.detection_service("")
    rospy.spin()

