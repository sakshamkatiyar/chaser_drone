#!/usr/bin/env python

'''
* Team Id :		534
* Author List :		Saksham Katiyar, Prankur Verma, Rishabh Handa, Saumya Singhal
* Filename:		eYRC#534.py
* Theme:		Chaser Drone
* Functions:		__init__(self)
			getMarkerPose(self, data)
			service_check(self, target, thresh_value)
			velocity(self)
			motion(self)
			identify_key(self, msg)
			arm(self)
			disarm(self)
			pitch(self, vel)
			roll(self, vel)
			throttle(self, vel)
			land(self)
			control_drone(self)
* Global Variables:	waypts[][]
			pt
			flag[]
			markers
			cave_time
			caved
			prev_time
			thresh
			min_vel
			max_vel
			current_position[]
			errSum[]
			prev_position[]
			prev_vel[]
			kp
			ki
			kd
'''

##import required libraries
import math
import time
import rospy

##import message and service types
from plutodrone.msg import *
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray
from whycon.srv import SetNumberOfTargets

##setting global variables for maintaining standard values
waypts = [[0.0, 0.7,25],								#HOME
          [ 7,-7,26],									#LN1
          [ 7, 6,26],									#LN5
          [0.0, 0.7,25],								#HOME
          [-5,-5,28]]									#Runner
pt = 0											#Index of Node to be traversed
flag = [1,1,1]										#Flag variable for error check
markers = 2										#Number of markers detected
cave_time = time.time()									#Time till which marker is detected
caved = time.time() * 10								#Time since runner is in cave
prev_time = time.time()									#Previous time for calculation of integral and derivative term
sample_time = 0.008
thresh = 1.0										#Maximum allowed position error
min_vel = -15										#Minimun velocity of chaser
max_vel = 15										#Maximum velocity of chaser
current_position = [-0.0,0.7,34]							#Present location of chaser
errSum = [0,0,0]									#Integral sum of error
prev_position = [-0.0,0.7,34]								#Last known location of chaser
prev_vel = [0,0,0]									#Storing previous velocity value
output = [0,0,0]									#List containing magnitude of velocities in all axes

kp = 1											#Proportional gain for x,y,z error
ki = 0											#Integral gain for x,y,z error
kd = 0											#Derivative gain for x,y,z error

class send_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_server')
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		rospy.Subscriber('/input_key', Int16, self.indentify_key )
		rospy.Subscriber('whycon/poses', PoseArray, self.getMarkerPose)		#Declaring that the node subscribes to the whycon/poses topic which is of type geometry_msgs.msg.PoseArray
		
		self.key_value = 0
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000

	'''
	*Function Name:	getMarkerPose
	*Input:		data-> ros message containing pose of the whycon marker i.e., the drone
	*Output:	None
	*Logic:		Updates the value of current location of the chaser and runner
	*Example call:	rospy.Subscriber('whycon/poses', PoseArray, self.getMarkerPose)
	'''
	def getMarkerPose(self, data):
		self.pose = data
		global waypts								#Declaring the variable global
		global current_position							#Declaring the variable global
		global cave_time							#Declaring the variable global

		if markers == 1:														#If 1 marker is detected
			current_position = [self.pose.poses[0].position.x, self.pose.poses[0].position.y, self.pose.poses[0].position.z]	#Updating the value of chaser

#		if markers == 2:														#If 2 markers are detected
#			waypts[4] = [self.pose.poses[0].position.x, self.pose.poses[0].position.y, self.pose.poses[0].position.z]		#Uppdating the value of runner
#			current_position = [self.pose.poses[1].position.x, self.pose.poses[1].position.y, self.pose.poses[1].position.z]	#Updating the value of chaser

#			d0 = math.sqrt(math.pow(prev_position[0]-waypts[4][0],2) + math.pow(prev_position[1]-waypts[4][1],2) + math.pow(prev_position[2]-waypts[4][2],2))	#Distance between runner and last location
#			d1 = math.sqrt(math.pow(prev_position[0]-current_position[0],2) + math.pow(prev_position[1]-current_position[1],2) + math.pow(prev_position[2]-current_position[2],2))	#Distance between chaser and last location
#			if d0<d1:							#Comparing change in last location
#				temp = waypts[4]					#Interchanging runner and chaser
#				waypts[4] = current_position				#Interchanging runner and chaser
#				current_position = temp					#Interchanging runner and chaser
#			waypts[4][2] -= 2.5						#Updating height for chaser

		cave_time = time.time()							#Updating last time of tracking

	'''
	*Function Name:	service_check
	*Input:		target       -> count of markers to be detected
			thresh_value -> threshold value for detection
	*Output:	targets      -> returns current targets value on which algo is working
	*Logic:		Changes the value of markers being detected in runtime
	*Example call:	markers = self.service_check(1,0)
	'''
	def service_check(self, target, thresh_value):
		rospy.wait_for_service('whycon/reset')
		try:
			target_value = rospy.ServiceProxy('whycon/reset', SetNumberOfTargets)
			resp1 = target_value(target,thresh_value)
			return resp1.targets
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	'''
	*Function Name:	velocity
	*Input:		None
	*Output:	output -> returns a list containing magnitude of the velocity on all axes
	*Logic:		Calculates the value of velocity on each axis using PID
			Limits the value of maximum and minimum velocity for smoother flight
	*Example call:	value = self.velocity()
	'''
	def velocity(self):
		global prev_time							#Declaring the variable global
		global errSum								#Declaring the variable global
		global output
		now = time.time()							#Current time in secondes
		timeChange = now - prev_time						#Time difference in seconds since last calculation
		for i in xrange(3):							#Looping in all axes
			error = waypts[pt][i] - current_position[i]			#Error in position

			if(prev_position[i]!=current_position[i]):			#Checking if chaser is at a distance and moving
				errSum[i] += error * timeChange				#Integral value of PID
				if errSum[i]>max_vel:					#Limiting the maximum value of integral error correction
					errSum[i] = max_vel				#Updating the integral error correction
				elif errSum[i]<min_vel:					#Limiting the minimum value of integral error correction
					errSum[i] = min_vel				#Updating the integral error correction

				dInput = (current_position[i] - prev_position[i]) / timeChange		#Derivative value of PID
			else:
				dInput = 0						#Set derivative error to zero

			output[i] = kp * error + ki * errSum[i] - kd * dInput		#Output velocity
			output[i] = math.fabs(output[i])				#Taking absolute value of output to get magnitude of velocity
			if output[i]>max_vel:						#Limiting the maxmum value of output
				output[i] = max_vel					#Updating the output

			prev_position[i] = current_position[i]				#Updating previous position
		prev_time = now								#Updating previous time

	'''
	*Function Name:	motion
	*Input:		None
	*Output:	None
	*Logic:		Calculates the required PID in respective direction to move in order to reduce error on each axis
			Check if the runner is in cave
			Land the chaser over runner
	*Example call:	self.motion()
	'''
	def motion(self):
		global pt								#Declaring the variable global
		global flag								#Declaring the variable global
		global caved								#Declaring the variable global
		global errSum								#Declaring the variable global
		global markers								#Declaring the variable global
		global prev_vel								#Declaring the variable global
		global cave_time							#Declaring the variable global

#		if time.time() - cave_time >= 1:					#Threshold for time detection
#			markers = self.service_check(1,0)				#Updating marker count
#			caved = time.time()						#Starting timer for cave
#		if time.time() - caved >= 16:						#If specified time has passed
#			markers = self.service_check(2,0)				#Updating marker count
#			caved = time.time() * 10					#Reseting timer value

#		print pt

		if (waypts[pt][0]+thresh <= current_position[0]):			#Checking if the drone is in positive side on x-axis
			flag[0] = -1							#Negative x error
			self.pitch(-25)						#Give negative PID value to drone x-axis
		elif (current_position[0] <= waypts[pt][0]-thresh):			#Checking if the drone is in negative side on x-axis
			flag[0] = 1							#Positive x error
			self.pitch(15)						#Give positive PID value to drone x-axis
		else:									#If x coordinate is in range
			flag[0] = 0							#No x error
			errSum[0] = 0							#Reset errSum for x axis
			self.pitch(0)							#No motion on x-axis motion

		if (waypts[pt][1]+thresh <= current_position[1]):			#Checking if the drone is in positive side on y-axis
			flag[1] = -1							#Negative y error
			self.roll(-15)						#Give negative PID value to drone y-axis
		elif (current_position[1] <= waypts[pt][1]-thresh):			#Checking if the drone is in positive side on y-axis
			flag[1] = 1							#Positive y error
			self.roll(15)						#Give positive PID value to drone y-axis
		else:									#If y coordinate is in range
			flag[1] = 0							#No y error
			errSum[1] = 0							#Reset errSum for y axis
			self.roll(0)							#No motion on y-axis motion

		if (waypts[pt][2]+thresh <= current_position[2]):			#Checking if the drone is in positive side on z-axis
			flag[2] = 1							#Positive z error
			self.throttle(200)					#Give positive PID value to drone z-axis
		elif (current_position[2] <= waypts[pt][2]-thresh):			#Checking if the drone is in positive side on z-axis
			flag[2] = -1							#Negative z error
			self.throttle(-40)					#Give negative PID value to drone z-axis
		else:									#If z coordinate is in range
			flag[2] = 0							#No z error
			errSum[2] = 0							#Reset errSum for z axis
			self.throttle(0)						#No motion on z-axis motion

		if flag == [0,0,0]:							#If no error in all axis
			pt+=1								#Update index location of next point
			if pt >= 5:
				pt = 4
				if marker == 2:						#If last point is traversed
					print 'momentum'
					for i in xrange(5000):				#Creating a small timer
						self.pitch(-prev_vel[0])		#Apply velocity in opposite x to decrease momentum
						self.roll(-prev_vel[1])			#Apply velocity in opposite y to decrease momentum
						self.throttle(0)			#Hover chaser at same altitude
					self.land()					#Land the chaser

		prev_vel = output							#Update last velocity value

	def indentify_key(self, msg):
		self.key_value = msg.data

	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)
	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def pitch(self, vel):								#Give motion along y-axis
		self.cmd.rcPitch = 1500 + vel						#Set pitch to desired value
		self.command_pub.publish(self.cmd)					#Send updated value to drone
	def roll(self, vel):								#Give motion along x-axis
		self.cmd.rcRoll = 1500 + vel						#Set roll to desired value
		self.command_pub.publish(self.cmd)					#Send updated value to drone
	def throttle(self, vel):							#Give motion along z-axis
		self.cmd.rcThrottle = 1500 + vel					#Set throttle to desired value
		self.command_pub.publish(self.cmd)					#Send updated value to drone

	'''
	*Function Name:	land
	*Input:		None
	*Output:	None
	*Logic:		Land the chaser
	*Example call:	self.land()
	'''
	def land(self):
		print 'land'
		for i in xrange(10000):							#Creating a small timer
			self.pitch(0)							#Set x motion to zero
			self.roll(0)							#Set y motion to zero
			self.throttle(-100)						#Decrease altitude
		print 'Caught Runner'							#Print Solution
		while(True):								#Infinte loop
			self.disarm()							#Disarm drone

	def control_drone(self):
		while True:
			if self.key_value == 0:
				self.disarm()
			if self.key_value == 10:
				self.arm()
			if self.key_value == 20:
				self.motion()
			if self.key_value == 30:
				self.pitch(0)
				self.roll(0)
				self.throttle(-200)
			self.command_pub.publish(self.cmd)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = send_data()
		test.control_drone()
		rospy.spin()
		sys.exit(1)

