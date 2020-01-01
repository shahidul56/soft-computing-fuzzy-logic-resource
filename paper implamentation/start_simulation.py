#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from rosgraph_msgs.msg import Clock
from kobuki_msgs.msg import MotorPower
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
import tf.transformations

from fuzzy_calculation import FuzzyImplementation

angularDiscretizations = [106, 319, 532]

goal = (3.8,7.0)

class TurtleBot_ROS():

	rangeValues = []
	fuzzy = FuzzyImplementation(goal)

	OAFLC_linearVelocity = 0.0
	OAFLC_angularVelocity = 0.0

	TFLC_linearVelocity = 0.0
	TFLC_angularVelocity = 0.0

	currentPosition = [0.0, 0.0]

	prevValues = [10.0, 10.0, 10.0]
	nanReplacer = [10.0, 10.0, 10.0]

	def __init__(self, node_name, update_freq):

		rospy.init_node(node_name, anonymous=False)
		rospy.loginfo("To stop the simulation CTRL + C")
		rospy.on_shutdown(self.shutdown)
		self.rate = rospy.Rate(update_freq)
		self.command = Twist()


	def create_publishers(self):
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.safety = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size=10)

		self.motor_power = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
		self.velocity = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

	# Defining the subscribers

	def create_subscribers(self):
		self.joint_states = rospy.Subscriber('/joint_states', JointState, self.callback_jointstates)
		self.robot_odom = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
		self.laserscan = rospy.Subscriber('/scan', LaserScan, self.callback_laserscan)
		self.gazebo_modelstates = rospy.Subscriber('/gazebo/model_states', LinkStates, self.callback_model_states )

	# Defining Callbacks

	def callback_jointstates(self, data):
		self.jointstate_data = data

	def callback_odometry(self, data):
		
		self.odomData = data


	def callback_laserscan(self, data):
		self.laserscan_data = data

		iterator = 0
		rangeValue = 0.0

		self.rangeValues = []
		for discretization in angularDiscretizations:
			
			val = data.ranges[discretization]

			if math.isnan(val):
				if self.prevValues[iterator]<0.6:
					self.nanReplacer[iterator] = 0.0
				else:
					self.nanReplacer[iterator] = 10.0
				
				rangeValue = self.nanReplacer[iterator]
				# self.rangeValues.append(self.nanReplacer[iterator])
			else:

				minVal = 10.0
				for val in data.ranges[(discretization-106):(discretization+106)]:
					if not math.isnan(val):
						if val<minVal:
							minVal = val
				rangeValue = minVal

			self.prevValues[iterator] = rangeValue

			self.rangeValues.append(rangeValue)

			iterator += 1

		self.rangeValues.reverse()
		# print self.rangeValues

		self.OAFLC_linearVelocity, self.OAFLC_angularVelocity = self.fuzzy.membership_functions_OAFLC(self.rangeValues)


	def callback_model_states(self, data):
		
		turtlebotPointer = len(data.name)-1

		pos = data.pose[turtlebotPointer].position
		self.currentPosition = [pos.x, pos.y]

		ori = data.pose[turtlebotPointer].orientation
		orientationQuaternions = (ori.x, ori.y, ori.z, ori.w)
		self.orientation = tf.transformations.euler_from_quaternion(orientationQuaternions)[2]*(180/np.pi)

		posError, angError = self.fuzzy.getOdometryErrors(self.currentPosition, self.orientation)
		self.TFLC_linearVelocity, self.TFLC_angularVelocity = self.fuzzy.membership_functions_TFLC(posError, angError)


	# Simplifying motion commands
	def move(self, OAFLC_weight):
		self.command.linear.x = OAFLC_weight*self.OAFLC_linearVelocity + (1-OAFLC_weight)*self.TFLC_linearVelocity				#Forward motion in m/s
		self.command.linear.y = 0								#Zero since the robot has non-holonomic constraints 
		self.command.angular.z = OAFLC_weight*self.OAFLC_angularVelocity + (1-OAFLC_weight)*self.TFLC_angularVelocity			#Angular Speeds in radians/s

		self.cmd_vel.publish(self.command)

	def stop_bot(self):
		self.cmd_vel.publish(Twist())

	def shutdown(self):
			# stop turtlebot
			rospy.loginfo("Stop TurtleBot")
			# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.stop_bot()
			# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

def main():

	Tbt = TurtleBot_ROS('TurtleBotFLC', 10)
	Tbt.create_publishers()
	Tbt.create_subscribers()

	# -ve angular velocity for Right turn
	# +ve angular velocity for Left turn

	while not rospy.is_shutdown():

		print "TFLC Velocity", Tbt.TFLC_linearVelocity, Tbt.TFLC_angularVelocity
		print "OAFLC Velocity", Tbt.OAFLC_linearVelocity, Tbt.OAFLC_angularVelocity,"\n"
		print "Current Position", Tbt.currentPosition

		if abs(Tbt.currentPosition[0]-goal[0])<0.1 and abs(Tbt.currentPosition[1]-goal[1])<0.1:
			break

		Tbt.move(OAFLC_weight=0.25)

if __name__ == '__main__':
		main()
