import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, 
    durability=DurabilityPolicy.VOLATILE, 
    history=HistoryPolicy.KEEP_LAST, 
    depth=10)

# For indexing X
X=0
Y=1
TH=2
W=3
V=4
VDOT=5

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # DONE Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        # n x 1 = 6 x 1
        x= np.array([
            [0],    # x
            [0],    # y
            [0],    # th
            [0],    # w
            [0],    # v
            [0]     # vdot (a)
        ])
        #value of Q given in part 4, multiplied by identity matrix
        # n x n = 6 x 6
        Q= np.eye(6)*0.5
        # Q= np.array([
        #     [0.5, 0, 0, 0, 0, 0], 
        #     [0, 0.5, 0, 0, 0, 0], 
        #     [0, 0, 0.5, 0, 0, 0], 
        #     [0, 0, 0, 0.5, 0, 0], 
        #     [0, 0, 0, 0, 0.5, 0], 
        #     [0, 0, 0, 0, 0, 0.5]
        # ])
        #value of R given in part 4, multiplied by identity matrix
        # m x m = 4 x 4
        R= np.eye(4)*0.5
        # R= np.array([
        #     [0.5, 0, 0, 0],
        #     [0, 0.5, 0, 0],
        #     [0, 0, 0.5, 0],
        #     [0, 0, 0, 0.5]
        # ])
        
        P=Q #TA said you could initialize to Q
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # DONE Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # DONE Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z
        acceleration_x = imu_msg.linear_acceleration.x
        acceleration_y = imu_msg.linear_acceleration.y
        
        # m x 1 = 4 x 1
        z=np.array([
            [linear_velocity],
            [angular_velocity],
            [acceleration_x],
            [acceleration_y]
        ])
        
        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        x_hat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose = np.array([
            x_hat[0], 
            x_hat[1],
            x_hat[2],
            imu_msg.header.stamp
        ])

        # DONE Part 4: log your data
        # ["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]
        self.loc_logger.log_values([
            imu_msg.linear_acceleration.x,  # imu_ax
            imu_msg.linear_acceleration.y,  # imu_ay
            x_hat[VDOT].item(),                    # kf_ax
            (x_hat[V] * x_hat[W]).item(),            # kf_ay
            x_hat[V].item(),                       # kf_vx
            x_hat[W].item(),                       # kf_w
            x_hat[X].item(),                       # kf_x
            x_hat[Y].item(),                       # kf_y
            imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec*(1e-9) 
            #imu_msg.header.stamp.to_sec()   # timestamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
