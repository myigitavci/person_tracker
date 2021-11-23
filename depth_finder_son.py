#!/usr/bin/env python3
import cv2
import re
import numpy as np
import jetson.inference
import jetson.utils
import argparse
import sys
import os
import io
import time
import json
import random
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Point

rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('chatter', Point, queue_size=10)
rate = rospy.Rate(10) # 10hz


# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("--network", type=str, default="ssd-mobilenet-v2",help="pre-trained model to load (see below for options)")

parser.add_argument("--threshold", type=float, default=0.5,help="minimum detection threshold to use")

parser.add_argument("--width", type=int, default=640,help="set width for image")

parser.add_argument("--height", type=int, default=480,help="set height for image")

opt = parser.parse_known_args()[0]
oldtime = time.time()

def depth_finder():
  #out = cv2.VideoWriter('home/desktop/yigitoutput.avi', -1, 20.0, (640,480))
 global pipeline
 out = cv2.VideoWriter('output6.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 60, (640,480))
 
# load the object detection network
 net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)
 center_old=[0,0]
 box_center=[0,0]
 temp=500
# Configure depth and color streams

 pipeline = rs.pipeline()
 config = rs.config()
 config.enable_stream(rs.stream.depth, opt.width, opt.height, rs.format.z16, 30)
 config.enable_stream(rs.stream.color, opt.width, opt.height, rs.format.bgr8, 30)

# Start streaming

 pipeline.start(config)

 os.system("sudo sh -c 'echo 128 > /sys/devices/pwm-fan/target_pwm'")

 press_key = 0
 while (press_key==0) :
	
   # Wait for a coherent pair of frames: depth and color

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue	
   # Convert images to numpy arrays

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    show_img = np.asanyarray(color_frame.get_data())
   
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)


   # convert to CUDA (cv2 images are numpy arrays, in BGR format)
    bgr_img = jetson.utils.cudaFromNumpy(show_img, isBGR=True)

	
   # convert from BGR -> RGB
    img = jetson.utils.cudaAllocMapped(width=bgr_img.width,height=bgr_img.height,format='rgb8')

    jetson.utils.cudaConvertColor(bgr_img, img)

   # detect objects in the image (with overlay)
 
    detections = net.Detect(img)
   
    k=0
    if 1==1:
     center_old=box_center
    
     person_count=0
     for i in range(len(detections)) :
     
      if detections[i].ClassID==1:
       fark=(round((center_old[0]-detections[i].Center[0]))**2+round((center_old[1]-detections[i].Center[1]))**2)/1000
       person_count=person_count+1
       print('insan')
       print('fark=',fark,' temp=',temp)
       if abs(fark)<=abs(temp)+10:
        temp=fark
        num=i
        print(person_count)
        score = round(detections[num].Confidence,2)
        box_top=int(detections[num].Top)
        box_left=int(detections[num].Left)
        box_bottom=int(detections[num].Bottom)
        box_right=int(detections[num].Right)
        box_center = detections[num].Center
        label_name = net.GetClassDesc(detections[num].ClassID)



     #if person_count==0 or detections[i].ClassID!=1 :
     # print(person_count,'2.if')   
  # for num in range(len(detections)) :
    #if detections[num].ClassID==1:
      #score = round(detections[i].Confidence,2)
     # box_top=int(detections[i].Top)
     # box_left=int(detections[i].Left)
      #box_bottom=int(detections[i].Bottom)
      #box_right=int(detections[i].Right)
     # box_center = detections[i].Center
      #label_name = net.GetClassDesc(detections[i].ClassID)

     point_distance=0.0
     for i in range (10):
        if len(detections)>0: 
         point_distance =  depth_frame.get_distance(int(box_center[0]),int(box_center[1]))

         point_distance = np.round(point_distance , 3)

         distance_text = str(point_distance) + 'm  ' + str(score)+ '%'

         cv2.rectangle(show_img,(box_left,box_top),(box_right,box_bottom),(255,0,0),2)

         cv2.line(show_img,(int(box_center[0])-10, int(box_center[1])),(int(box_center[0]+10), int(box_center[1])),(0, 0, 255), 3)

         cv2.line(show_img,(int(box_center[0]), int(box_center[1]-10)),(int(box_center[0]), int(box_center[1]+10)),(0, 0, 255), 3)

         cv2.putText(show_img,label_name + ' ' + distance_text,(box_left+5,box_top+20),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1,cv2.LINE_AA)

         cv2.putText(show_img,"{:.0f} FPS".format(net.GetNetworkFPS()),(int(opt.width*0.8), int(opt.height*0.1)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)

        display = cv2.resize(show_img,(int(opt.width*1.5),int(opt.height*1.5)))

        images = np.hstack((color_image, depth_colormap))
        out_vid=cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
        cv2.imshow('RealSense', images)
        out.write(out_vid)
    pt = Point(x = box_center[0], y = box_center[1],z=point_distance)
    pub.publish(pt)  
    keyValue=cv2.waitKey(1)
    if keyValue & 0xFF == ord('q') or time.time() - oldtime >30:
        press_key=1
        out.release()
 
# 
depth_finder()

cv2.destroyAllWindows()
pipeline.stop()
