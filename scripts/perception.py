#!/usr/bin/env python

import rospy
import random
import numpy as np
import math
import sys
import sensor_msgs.point_cloud2 as pc2
from random import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from laser_geometry import LaserProjection
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global r_goal
r_goal = False
global h_wall
h_wall = False
global s_wall
s_wall = False
global laserPoints
laserPoints = [[]]
global robot_position
robot_position = [-8.0, -2.0]
global modelpoint1
modelpoint1 = [0, 0]
global modelpoint2
modelpoint2 = [0, 0]
global model_grad
model_grad = 0
global seen_wall
seen_wall = False
global likely_corner
likely_corner = False
global roll, pitch, yaw
roll = pitch = yaw = 0.0

def find_line(p1, p2):
      m = (p2[1] - p1[1])/(p2[0] - p1[0] + sys.float_info.epsilon)
      c = p2[1] - m * p1[0]
      return m, c

def find_intercept_point(m, c, x0, y0):
      x = (x0 + m*y0 - m*c)/(1 + m**2)
      y = (m*x0 + (m**2)*y0 - (m**2)*c)/(1 + m**2) + c
      return x, y

def ransac(data):
      #ref = https://salzis.wordpress.com/2014/06/10/robust-linear-model-estimation-using-ransac-python-implementation/
      ransac_iterations = 50
      ransac_threshold = 0.1
      ransac_ratio = 0.5

      ratio = 0
      model_m = 0
      model_c = 0

      m_p1 = []
      m_p2 = []

      for i in range(ransac_iterations):
            rand_2_p = np.sort(np.random.randint(0, len(data), 2))
            p1 = data[rand_2_p[0]]
            p2 = data[rand_2_p[1]]
            m, c = find_line(p1, p2)
            
            x_list = []
            y_list = []
            p_list = []
            num = 0

            for dp in data:
                  x0 = dp[0]
                  y0 = dp[1]
                  x1, y1 = find_intercept_point(m, c, x0, y0)
                  dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
                  if dist < ransac_threshold:
                        #x_list.append(x0)
                        #y_list.append(y0)
                        p_list.append(dp)
                        num += 1
            #x_inlier = np.array(x_list)
            #y_inlier = np.array(y_list)

            if num/float(len(data)) > ratio:
                  ratio = num/float(len(data))
                  model_m = m
                  model_c = c
                  m_p1 = p_list[0]
                  m_p2 = p_list[-1]


            if num > len(data)*ransac_ratio:
                  m_p1 = p_list[0]
                  m_p2 = p_list[-1]
                  break

      if not m_p1 and not m_p2:
            m_p1 = [0, 0]
            m_p2 = [0, 0]

      return m_p1, m_p2

def callback(data):
      global h_wall
      h_wall = False
      global s_wall
      s_wall = False
      global model_grad
      #global seen_wall
      m = (modelpoint2[1] - modelpoint1[1])/(modelpoint2[0] - modelpoint1[0] + sys.float_info.epsilon)
      model_grad = m
      for i in range(361):
              if data.ranges[i] <= 0.5 :
                   h_wall = True
              if data.intensities[i] == 1.0:
                   s_wall = True
      #if s_wall == False and seen_wall:
              #seen_wall = False

def laserCallback(data):
      global laserPoints
      global modelpoint1
      global modelpoint2
      laserPoints = []
      cloud_out = LaserProjection().projectLaser(data)
      for p in pc2.read_points(cloud_out, field_names = ("x", "y"), skip_nans=True):
            laserPoints.append([p[0],p[1]])
      if laserPoints:
            modelpoint1, modelpoint2 = ransac(laserPoints)
      else:
            modelpoint1 = [0, 0]
            modelpoint2 = [0, 0]

def odometryCallback(msg):
      global robot_position
      global roll, pitch, yaw
      robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
      robot_orientation = [0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
      (roll, pitch, yaw) = euler_from_quaternion(robot_orientation)


def robot0():
      rospy.init_node('robot0', anonymous=True)
      #global pc_pub
      #pc_pub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
      rospy.Subscriber("base_scan", LaserScan, laserCallback)
      rospy.Subscriber("base_scan", LaserScan, callback)
      rospy.Subscriber("/base_pose_ground_truth", Odometry, odometryCallback)
      robot0_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
      line_pub = rospy.Publisher("/ransac_line", Marker, queue_size=1)
      
      global seen_wall
      rdi = [-1,1]
      rotate_dir = sample(rdi,1)[0]
      on_track = True
      right_angle = True
      cor_angle = False
      cal_angle = False
      walltoTrijectory = False
      move_tw = False
      f_wall = True
      trail_wall = False
      r_direction = quaternion_from_euler (0.0, 0.0, math.atan(2))

      rate = rospy.Rate(100)
      rospy.sleep(0.2)
      while not rospy.is_shutdown():
            line_marker=Marker()
	    line_marker.header.frame_id = "base_laser_link"
	    line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.color.a = 1
	    line_marker.color.g = 1
	    line_marker.lifetime = rospy.Duration(0)
	    line_marker.scale.x = 0.05
            #line_marker.pose.orientation.z = line_orientation[0]
            #line_marker.pose.orientation.w = line_orientation[1]
	    p1 = Point()
	    p1.x = modelpoint1[0]
	    p1.y = modelpoint1[1]
	    p2 = Point()
	    p2.x = modelpoint2[0]
	    p2.y = modelpoint2[1]
	    line_marker.points.append(p1)
	    line_marker.points.append(p2)
	    line_pub.publish(line_marker)

            """i_x, i_y  = find_intercept_point(0.8, 4.4, robot_position[0], robot_position[1])
            p_f_l = math.atan((i_y-robot_position[1])/(i_x-robot_position[0]+sys.float_info.epsilon))
            c_y = 0.8*robot_position[0]+4.4
            end_position = [4.5, 9.0]
            print(model_grad)

            twist = Twist()

            if round(robot_position[0], 1) == 4.5 and round(robot_position[1], 1) == 9.0:
                  print(robot_position) 
            elif s_wall == True:
                  if (round(c_y, 1) - 0.1) <= round(robot_position[1], 1) <= (round(c_y, 1) + 0.1) and not walltoTrijectory:
                        walltoTrijectory = True
                        cal_angle = True
                  input_tan = (end_position[1]-robot_position[1])/(end_position[0]-robot_position[0]+sys.float_info.epsilon)
                  if not round(yaw, 2) == round(math.atan(input_tan), 2) and cal_angle:
                        twist.angular.z = -(round(yaw, 2) - round(math.atan(input_tan), 2))
                  else:
                        cal_angle = False
                        move_tw = True
                  if h_wall and f_wall:
                        f_wall = False
                        seen_wall = True
                        trail_wall = True
                        rotate_dir = -1
                  elif h_wall:
                        if not -0.1 < model_grad < 0.1:
                              twist.angular.z = rotate_dir*0.1
                        elif trail_wall and (-0.1 > model_grad > 0.1) and (not (round(c_y, 1) - 0.1) <= round(robot_position[1], 1) <= (round(c_y, 1) + 0.1)):
                              twist.angular.z = -(round(yaw, 2) - round(math.atan(input_tan), 2))
                        elif move_tw == True:
                              twist.linear.x = 2.0;
                  elif move_tw == True:
                        twist.linear.x = 2.0;
            #elif s_wall == False:
                  #if seen_wall and (not (round(c_y, 1) - 0.1) <= round(robot_position[1], 1) <= (round(c_y, 1) + 0.1)):
                        #twist.angular.z = -(round(yaw, 2) - round(math.atan(input_tan), 2))
                  #elif move_tw == True:
                        #twist.linear.x = 2.0;
            elif move_tw == True:
                        twist.linear.x = 2.0;
            elif round(robot_position[1], 1) > round(0.8*robot_position[0]+4.4, 1):
                  if not round(yaw, 2) == round(p_f_l, 2):
                        print("g")
                        cor_angle = False
                        on_track = False
                  else:
                        cor_angle = True
                        on_track = True
                 	print("greater: ", robot_position[1])
                  	rotate_dir = -1
            elif round(robot_position[1], 1) < round(0.8*robot_position[0]+4.4, 1):
                  if not round(yaw, 2) == round(p_f_l, 2):
                        print("l")
                        cor_angle = False
                        on_track = False
                  else:
                        cor_angle = True
                        on_track = True
                        print("lesser: ", robot_position[1])
                        rotate_dir = 1
            elif not round(yaw, 2) == round(math.atan(2), 2):
                  print(math.degrees(round(yaw, 2)))
                  right_angle = False
            else:
                  on_track = True
                  right_angle = True

            twist = Twist()
            if h_wall == False and on_track == True and right_angle == True and cor_angle == False:
                  print("Moving")
                  twist.linear.x = 1.0;
                  rotate_dir = sample(rdi,1)[0]
            elif h_wall == True:
                  twist.angular.z = rotate_dir*0.1#uniform(0.1,0.5);
            elif on_track == False:
                  twist.angular.z = -(round(yaw, 2)-round(p_f_l, 2))
                  print(-(round(yaw, 2)-round(p_f_l, 2)))
            elif right_angle == False:
                  twist.angular.z = -(round(yaw, 2)-round(math.atan(2), 2))
            elif cor_angle == True:
                  twist.linear.x = math.sqrt((i_x-robot_position[0])**2 + (i_y-robot_position[1])**2);

            robot0_pub.publish(twist)"""

            rate.sleep()

if __name__ == '__main__':
      robot0()
