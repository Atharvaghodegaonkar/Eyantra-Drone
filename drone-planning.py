#!/usr/bin/env python

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
		self.j = 0 # vairiable for the storing the position 
		# [x_setpoint, y_setpoint, z_setpoint]
		#self.setpoint = [2,2,20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.prev_error = [0,0,0] # Stores the previous error
		self.op_error = [0,0,0]   # Stores the output error
		self.error = [0,0,0]      # Store the error value
		self.min_values = [1000,1000,1000] # Minimum value for drone  
		self.max_values = [2000,2000,2000] # maximum value for drone 
		self.error_sum = [0,0,0]  
		self.interm = [0,0,0]
		self.setpoint_list = ([0.2, 0, 23], [3.5, -3.3, 23], [-4.4, -4.6, 20.7] , [-6.0, 6.3, 18.3], [5.0, 5.3, 16.3]) #Co ordinate list
		self.current_time = 0.00 # stores the time 
		self.prev_time = 0.00
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [22,22,55]
		self.Ki = [0.0004,0.0012,0.001]
		self.Kd = [825,825,980]


		
		self.sample_time = 0.033 # in seconds

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub= rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub= rospy.Publisher('/roll_error', Float64, queue_size=1)

		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		#rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		#rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		#rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		#------------------------------------------------------------------------------------------------------------

		#self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		#self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)

	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	def altitude_set_pid(self,alt):
		self.Kp[2] =  alt.Kp * 0.006 
		self.Ki[2] =  alt.Ki * 0.0001	
		self.Kd[2] =  alt.Kd * 0.24

	def pitch_set_pid(self,pit):
		self.Kp[1] = pit.Kp * 0.02 
		self.Ki[1] = pit.Ki * 0.0001
		self.Kd[1] = pit.Kd * 0.3

	def roll_set_pid(self,rol):
		self.Kp[0] = rol.Kp * 0.005 
		self.Ki[0] = rol.Ki * 0.0001
		self.Kd[0] = rol.Kd * 0.3

	#---------------------------------------------------------------------------------------------------------------------- 

	def pid(self):

		
			
		self.setpoint = self.setpoint_list[self.j]

		self.current_time = time.time()
		self.delta_time = self.current_time - self.prev_time

		if (self.delta_time > self.sample_time):
			for i in range (0,3):	

				self.error[i] = self.drone_position[i] - self.setpoint[i]                                                             #setpoint error
				self.interm[i] += (self.error[i]) * self.Ki[i]                                                                        # ki term calculation over the period of time
				self.op_error[i] = (self.Kp[i] * self.error[i]) + self.interm[i] + self.Kd[i] * (self.error[i] - self.prev_error[i])  #pid_error calculation
				self.prev_error[i] = self.error[i]
				self.prev_time = self.current_time
				if ( (self.setpoint[0] - 0.2) <= self.drone_position[0] <= (self.setpoint[0] + 0.2) and ((self.setpoint[1] - 0.2) <= self.drone_position[1] < (self.setpoint[1] + 0.2)) and ((self.setpoint[2] - 0.5) <= self.drone_position[2] <= (self.setpoint[2] + 0.5)) ):
					if self.j < 4:
						self.j = self.j + 1
						print('reached level{}'.format(self.j))
						break
					else :
						print('path complete')	
					
				    


			self.cmd.rcThrottle = 1500 + self.op_error[2]

			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]

			if self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]

			self.cmd.rcPitch = 1500 + self.op_error[1]
		
			if self.cmd.rcPitch > self.max_values[1]:
				self.cmd.rcPitch = self.max_values[1]

			if self.cmd.rcPitch < self.min_values[1]:
				self.cmd.rcPitch = self.min_values[1]	

			self.cmd.rcRoll = 1500 - self.op_error[0]

			if self.cmd.rcRoll > self.max_values[0]:
				self.cmd.rcRoll = self.max_values[0]

			if self.cmd.rcRoll < self.min_values[0]:
				self.cmd.rcRoll = self.min_values[0]	


				
					
	#------------------------------------------------------------------------------------------------------------------------


		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[1])
		self.roll_error_pub.publish(self.error[0])
		self.command_pub.publish(self.cmd)





if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
