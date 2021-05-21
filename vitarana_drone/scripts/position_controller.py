#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_control which controls the latitude longitude and altitude of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /edrone/drone_command   /edrone/gps
        /x_error		/edrone/range_finder_top
        /y_error                /edrone/range_finder_bottom
	/z_error

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import rospy
import time
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control
	
        # This corresponds to your current position of eDrone in quaternion format. This value must be updated each time in your gps callback
        # [lat, long, alt, orientation]
        self.drone_position = [0.0, 0.0, 0.0, 0.0]

        # This is the setpoint on which the drone is asked to go in the latitude
        # [lat_setpoint, long_setpoint, alt_setpoint, orientation_setpoint]
        self.setpoint = [19.0, 72.0, 4.0, 0.0]

	# This corresponds to the relative altitude of the eDrone from the surface below taken from the range_finder_bottom
	# For eg. if the drone is at 10m from the sea level but is actually 2m from the terrace of the building.
	self.altitude = 0.0

	# This is the setpoint for relative altitude
	self.altitude_setpoint = 0.0

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

	self.cmd = edrone_cmd()
	self.cmd.rcRoll = 0
	self.cmd.rcPitch = 0
	self.cmd.rcYaw = 0
	self.cmd.rcThrottle = 0
	self.cmd.aux1 = 0
	self.cmd.aux2 = 0
	self.cmd.aux3 = 0
	self.cmd.aux4 = 0        

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]. eg: self.Kp[2] corresponds to Kp value in altitude
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [0.0, 0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0, 0.0]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
	self.error = [0.0, 0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
	self.error_sum = [0.0, 0.0, 0.0, 0.0]

	# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
	self.prev_values = [0, 0, 0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]
	# ----------------------------------------------------------------------------------------------------------

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command, /x_error, /y_error, /z_error, /zero_error
        self.drone_command_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
	# -----------------------Add other ROS Publishers here--------------------------------------------------------
	self.roll_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
	self.pitch_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
	self.throttle_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
	self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)

	# ------------------------------------------------------------------------------------------------------------

        # Subscribing to /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw, /pid_tuning_altitude, /edrone/gps
	rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # ------------------------Add other ROS Subscribers here------------------------------------------------------
	rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps-callback)
        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def gps_callback(self, msg):

        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

        # ---------------------------------------------------------------------------------------------------------------

	# Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_latitude
    def latitude_set_pid(self, latitude):
        self.Kp[0] = latitude.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = latitude.Ki * 0.008
        self.Kd[0] = latitude.Kd * 0.3

    # ----------------------------Define callback function like latitude_set_pid to tune londitude_set_pid, altitude_set_pid--------------

    def londitude_set_pid(self, londitude):
        self.Kp[1] = longitude.Kp * 0.06
        self.Ki[1] = longitude.Ki * 0.008
        self.Kd[1] = longitude.Kd * 0.3

    def altitude_set_pid(self, altitude):
        self.Kp[2] = altitude.Kp * 0.06
        self.Ki[2] = altitude.Ki * 0.008
        self.Kd[2] = altitude.Kd * 0.3

	# ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the setpoint that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   4. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   6. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   8. Add error_sum to use for integral component

	self.error[0] = (self.setpoint[0] - self.drone_position[0])*1000000
	self.error[1] = (self.setpoint[1] - self.drone_position[1])*1000000
	self.error[2] = (self.setpoint[2] - self.drone_position[2])

	self.roll = 1500 + self.Kp[0]*self.error[0] + self.Ki[0]*self.error_sum[0] + self.Kd[0]*(self.error[0]-self.previous_error[0])
	self.pitch = 1500 + self.Kp[1]*self.error[1] + self.Ki[1]*self.error_sum[1] + self.Kd[1]*(self.error[1]-self.previous_error[1])
	self.throttle = 1500 + self.Kp[2]*self.error[2] + self.Ki[2]*self.error_sum[2] + self.Kd[2]*(self.error[2]-self.previous_error[2])

        #
        #
        #
        #
        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------

	self.previous_error[0]=self.error[0]
	self.previous_error[1]=self.error[1]
	self.previous_error[2]=self.error[2]

	self.error_sum[0] = self.error_sum[0] + self.error[0]
	self.error_sum[1] = self.error_sum[1] + self.error[1]
	self.error_sum[2] = self.error_sum[2] + self.error[2]

	self.cmd.rcRoll = self.roll
	self.cmd.rcPitch = self.pitch
	self.cmd.rcThrottle = self.throttle
        
	if(self.cmd.rcRoll>2000):
	    self.rcRoll=2000
	if(self.cmd.rcRoll<1000):
	    self.rcRoll=1000
	if(self.cmd.rcPitch>2000):
	    self.rcPitch=2000
	if(self.cmd.rcPitch<1000):
	    self.rcPitch=1000
	if(self.cmd.rcThrottle>2000):
	    self.rcThrottle=2000
	if(self.cmd.rcThrottle<1000):
	    self.rcThrottle=1000

	self.drone_command_pub.publish(self.cmd)
	self.roll_error_pub.publish(self.error[0])
	self.pitch_error_pub.publish(self.error[1])
	self.throttle_error_pub.publish(self.error[2])
	self.zero_error_pub.publish(0)

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()

