#!/usr/bin/env python3

#source: https://github.com/hsaeidi-uncw/ur5e_control.git, in clas slides, online resources
#Sidney Tsui
#Report 2

import rospy
import math

#outline from ur5e_control.git by hsaeidi-uncw

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot_vision_lectures.msg import SphereParams
#imports for camera to base transformations
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

raw_x = 0
raw_y = 0
raw_z = 0
rad = 0
curr = [0,0,0,0,0,0]

#processes data from ROS topic about the sphere

def get_sphere(data):
#global varrs can be changed through the entire node
	global raw_x, raw_y, raw_z, rad
	#assiging values to global varrs 
	raw_x = data.xc
	raw_y = data.yc
	raw_z = data.zc
	rad = data.radius
    
   	def add_pt(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, plan):
        add_point = Twist()#msg obj create
        add_point.linear.x = linear_x
        add_point.linear.y = linear_y
        add_point.linear.z = linear_z
        add_point.angular.x = angular_x
        add_point.angular.y = angular_y
        add_point.angular.z = angular_z
        add.points.append(add_point)#add plan_point to plan
		
	
	def get_curr(data):
		global curr#set as global to be updated later
		curr[0] = data.linear.x#update here
		curr[1] = data.linear.y
		curr[2] = data.linear.z
		curr[3] = data.angular.x
		curr[4] = data.angular.y
		curr[5] = data.angular.z

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	rospy.Subscriber('sphere_params', SphereParams, get_sphere_data)
	rospy.Subscriber('/ur5e/toolpose', Twist, get_curr)
 	#publisher
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()
	tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
	q_rot = Quaternion()
	#source: lect 20, slide 11-12, https://wiki.ros.org/tf2/Tutorials
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			#create message
			frame_pt = tf2_geometry_msgs.PointStamped()#init obj
    		frame_pt.header.frame_id = 'camera_color_optical_frame'#set ID
    		frame_pt.header.stamp = rospy.get_rostime()#timestamp
    		#assign points from stored values
    		frame_pt.point.x = raw_x
    		frame_pt.point.y = raw_y
    		frame_pt.point.z = raw_z
   	 		#point to base frame
    		new_pt = tfBuffer.transform(frame_pt, 'base', rospy.Duration(1.0))#camera to base
    		#get coord
    		base = tfBuffer.transform(ptcam,'base', rospy.Duration(1.0))
			x = base.point.x
			y = base.point.y
			z = base.point.z
  	  		radius = rad
  	  		q_rot = trans.transform.translation
  	  		print("x: ", x, "\n", "y: ", y,"\n", "z: ", z, "\n", "radius: ", radius)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
			
		
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
