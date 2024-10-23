# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once, shutdown
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class decision_maker(Node):
    # NOTE: Added default goalPoint
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint=[-1.0, -1.0], rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #DONE Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER)
        else:
            print("Error! you don't have this planner", file=sys.stderr)

        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        # NOTE: kept default goalPoint in planner in case other future nodes may use it? 
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        # DONE Part 3: Run the localization node
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg = Twist()
        reached_goal = False

        # DONE Part 3: Check if you reached the goal
        ang_error_tol = 0.1  # rad
        lin_error_tol = 0.05  # m
        if type(self.goal) == list:
            if (calculate_angular_error(self.localizer.getPose(), self.goal) < ang_error_tol and 
                calculate_linear_error(self.localizer.getPose(), self.goal) < lin_error_tol):
                reached_goal = True
        
        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # DONE Part 3: exit the spin
            raise SystemExit

        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # DONE Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)



import argparse


def main(args=None):
    
    init()

    # DONE Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    QoS = QoSProfile(depth=10)
    QoS.reliability = ReliabilityPolicy.BEST_EFFORT
    QoS.durability = DurabilityPolicy.VOLATILE
    QoS.history = HistoryPolicy.KEEP_LAST
    

    # DONE Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM = decision_maker(Twist, 'cmd_vel', QoS, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(Twist, 'cmd_vel', QoS, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)      
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
