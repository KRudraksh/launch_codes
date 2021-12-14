#!/usr/bin/env python2
import numpy as np
import rospy
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image,Imu
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped, Quaternion
import math
from time import sleep
ARM_RAD=1
class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()
		self.orient = Quaternion()
		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		#self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel,)
		# self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=0)
		self.local_pose_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')

	#MODE SETUP

	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		# self.gps_subscriber

		# t_lat = self.gps_lat
		# t_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	
	
	def land(self, l_alt):

		# self.gps_subscriber

		# l_lat = self.gps_lat
		# l_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/land')
		try:
			self.land_service(0.0, 0.0, 0, 0, l_alt)
			rospy.loginfo("LANDING")

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	def set_mode(self,md):

			rospy.wait_for_service('/mavros/set_mode')
			try:
				self.flight_mode_service(0, md)
				rospy.loginfo("Mode changed")
				
			except rospy.ServiceException as e:
				rospy.loginfo("Mode could not be set: " %e)

	def set_Guided_mode(self):
		
		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def set_Altitude_Hold_mode(self):

		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("ALT_HOLD")	

	#CALLBACKS

	# def gps_callback(self, data):
	# 	self.gps_lat = data.latitude
	# 	self.gps_long = data.longitude


	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

		# orientation in space  
		self.orient.x = location_data.pose.orientation.x
		self.orient.y = location_data.pose.orientation.y
		self.orient.z = location_data.pose.orientation.z
		self.orient.w = location_data.pose.orientation.w

	# def get_vel(self,vel_data):
	# 	self.x_vel=	vel_data.twist.linear.x
	# 	self.y_vel=	vel_data.twist.linear.y
	# 	self.z_vel=	vel_data.twist.linear.z
		
	def within_rad(self):
		# sp = PoseStamped()
		# # setting the orientation to the current orientation of the uav
		# sp.pose.orientation.x = self.orient.x
		# sp.pose.orientation.y = self.orient.y
		# sp.pose.orientation.z = self.orient.z
		# sp.pose.orientation.w = self.orient.w

		if (((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2) < (ARM_RAD)**2):
			return True
		print((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2)
		return False





	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z

		ix = self.orient.x
		iy = self.orient.y
		iz = self.orient.z
		iw = self.orient.w

		sp.pose.orientation.x = ix
		sp.pose.orientation.y = iy
		sp.pose.orientation.z = iz
		sp.pose.orientation.w = iw


		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.2):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)

	def local_gotopose(self,x,y,z):
		rate = rospy.Rate(20)		
		sp = PositionTarget()
		sp.header.frame_id = "map" # choose a frame_id (odom/map/base_link/world or any custom)
		sp.header.stamp = rospy.Time.now()
		sp.coordinate_frame = 1
		sp.type_mask = 4088 # ignoring all flags except position
		sp.position.x = x
		sp.position.y = y
		sp.position.z = z
		self.local_pose_publisher.publish(sp)
		
			

	def test_control(self):
		print('Starting')
		rate = rospy.Rate(20)
		sr = AttitudeTarget()
		sr.type_mask = 134
		sr.body_rate.x = 0.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 1
		for i in range(60):
			print('stg 1')
			self.publish_attitude_thrust.publish(sr)
			rate.sleep()
		print('Stage 1 done')
		



if __name__ == '__main__':

	mav = FLIGHT_CONTROLLER()
	time.sleep(3)
	print(mav.within_rad())
	if (mav.within_rad()):
		mav.set_mode('STABILIZE')
		mav.toggle_arm(1)
		time.sleep(3)
		mav.set_Guided_mode()
		mav.takeoff(5) #cannot takeoff using gotopose
		time.sleep(10)
		mav.local_gotopose(5,2,5)
		mav.land(5)
			
		mav.toggle_arm(0)

