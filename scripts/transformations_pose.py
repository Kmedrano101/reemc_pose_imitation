#!/usr/bin/env python3

"""Transformations_pose.py script to transform pose data into a angles for the reem-c robot

    Descriptions:
        Source needed: this script needs human_pose.py running in order to get the angles
        Main function: get the angles for the reem-c robot and set them to the param server
"""

# Import modules from ROS resources

import rospy
from rospy.exceptions import ROSException
from reemc_pose_imitation.msg import human_pose

# import modules from others resource

import time
from os import system
import numpy as np
import math

# Parameters and variables

try:
    # verify if parameters are available
    rospy.get_param_names()
except ROSException:
    print("[WARNING] can not get the parameters")

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "transformations_pose"
TOPIC_S1_POSE = PACKAGE_NAME+"/pose_human"

# Class Transformations


class Transformations:
    """Transformations class"""

    def __init__(self):
        self._subTopicHumanPoseName = None
        self.dataHumanPose = human_pose()
        self.paramPoints = []
        self.localPoints = []
        self.paramJointNames = []
        self.points = []
        self.webPoints = []
        self.posePoints = []
        self.imgWeight = 0
        self.imgHeight = 0

    """ Properties """
    @property
    def subTopicHumanPoseName(self):
        """The subTopicPersonPoseName property."""
        return self._subTopicHumanPoseName

    @subTopicHumanPoseName.setter
    def subTopicHumanPoseName(self, value):
        if value:
            self._subTopicHumanPoseName = value
        else:
            rospy.loginfo("Invalid Name of topic PersonPose")

    """ Methods and Actions"""

    def data_pose_callback(self, msg) -> None:
        self.dataHumanPose = msg

    def start_subscribers(self) -> None:
        """To start the subscribers"""
        rospy.Subscriber(self._subTopicHumanPoseName,
                        human_pose, self.data_pose_callback)

    def start_publishers(self) -> None:
        """No publishers needed"""
        pass

    def get_param_values(self) -> None:
        """Get the parameters from the parameter server"""
        # Get parameters from the yaml files
        self.paramJointNames = rospy.get_param('/play_motion/motions/my_move2/joints')
        self.paramPoints = rospy.get_param('/play_motion/motions/my_move2/points')
        self.imgWeight = rospy.get_param("/reemc_pose_imitation/image_source/weight")
        self.imgHeight = rospy.get_param("/reemc_pose_imitation/image_source/height")
        self.localPoints = rospy.get_param('/reemc_pose_imitation/local_points/points')
        # Get parameters from the web interface
        self.webPoints = rospy.get_param('/reemc_pose_imitation/web_points/points')
        self.mode = rospy.get_param("/reemc_pose_imitation/operation_mode", default=0)
        # Save values to a list 
        Points = list(self.paramPoints[0].values())
        self.points = Points[0]

    def set_param_values(self) -> None:
        """Set parameters to the param server"""
        AuxparamPoints = self.paramPoints[0]
        AuxparamPoints['positions'] = self.points
        self.paramPoints[0] = AuxparamPoints
        rospy.set_param('/play_motion/motions/my_move2/points',self.paramPoints)

    def get_angle(self, p1, p2, p3) -> float:
        """Get the angle from a given 3 points"""
        angle = 0
        points = []
        # First verify if there are poses data
        if self.dataHumanPose.points_pose:
            for pose in self.dataHumanPose.points_pose:
                # Get the x and y values according to the shape of the image
                h, w = self.imgHeight, self.imgWeight
                cx, cy = int(pose.position.x * w), int(pose.position.y * h)
                points.append([pose.id_pos, cx, cy])
            x1, y1 = points[p1][1:]
            x2, y2 = points[p2][1:]
            x3, y3 = points[p3][1:]
            # Calculate the Angle
            angle = math.degrees(math.atan2(y3 - y2, x3 - x2) -
                                math.atan2(y1 - y2, x1 - x2))
            if angle < 0:
                angle += 360
            self.posePoints = points
        return angle 

    def map_value(self,value,out_min,out_max,in_min,in_max) -> float: 
        """Map values to an other shape, the new output shape is according 
        to the limits of the robot angle"""
        value = float((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
        return value

    def get_distance(self,x1,x2,y1,y2) -> float:
        """Get distance between two (x,y) points"""
        distance = np.sqrt((x2-x1)**2+(y2-y1)**2)
        return distance

    def transformations(self) -> None:
        """Main function to transform the angles"""
        self.get_param_values()
        if self.mode == 0:
            if self.posePoints is not None:
                # Get angles for arm left joints
                angle_alfa1 = self.get_angle(13,11,23)
                angle_beta1 = self.get_angle(15,11,23)
                angle_gama1 = self.get_angle(15,13,11)
                self.localPoints[self.paramJointNames.index('arm_left_2_joint')] = round(self.map_value(angle_alfa1,0.0,2.0,0,120),2)
                if angle_beta1 > angle_alfa1:
                    self.localPoints[self.paramJointNames.index('arm_left_3_joint')] = 1.5
                else:
                    self.localPoints[self.paramJointNames.index('arm_left_3_joint')] = -1.5

                self.localPoints[self.paramJointNames.index('arm_left_4_joint')] = round(self.map_value(abs(185-angle_gama1),0.0,2.2,0.0,160.0),2)
                # Get angles for arm right joints
                angle_alfa2 = self.get_angle(24, 12, 14)
                angle_beta2 = self.get_angle(24, 12, 16)
                angle_gama2 = self.get_angle(12, 14, 16)
                self.localPoints[self.paramJointNames.index('arm_right_2_joint')] = round(self.map_value(angle_alfa2,0.0,2.0,0,120),2)
                if angle_beta2 > angle_alfa2:
                    self.localPoints[self.paramJointNames.index('arm_right_3_joint')] = 1.5
                else:
                    self.localPoints[self.paramJointNames.index('arm_right_3_joint')] = -1.5

                self.localPoints[self.paramJointNames.index('arm_right_4_joint')] = round(self.map_value(abs(185-angle_gama2),0.0,2.2,0.0,160.0),2) 
                # Get angles for head joints
                d1 = self.get_distance(self.posePoints[0][1],self.posePoints[7][1],self.posePoints[0][2],self.posePoints[7][2])
                d2 = self.get_distance(self.posePoints[0][1],self.posePoints[8][1],self.posePoints[0][2],self.posePoints[8][2])
                if d1 > d2:
                    # Validate limits to move the head joint to the right
                    if abs(d1-d2) < d1*0.58:
                        value1 = d1*0.58
                    else:
                        value1 = abs(d1-d2)
                    self.localPoints[self.paramJointNames.index('head_1_joint')] = -round(self.map_value(value1,0.0,1.6,0,(d1*0.58)),2)
                else:
                    # Validate limits to move the head joint to the left
                    if abs(d2-d1) < d2*0.58:
                        value2 = d2*0.58
                    else:
                        value2 = abs(d2-d1)
                    self.localPoints[self.paramJointNames.index('head_1_joint')] = round(self.map_value(value2,0.0,1.6,0,(d2*0.58)),2)
                for i in range(18):
                    self.points[i] = self.localPoints[i]
                self.set_param_values() 
        else:
            for i in range(18):
                self.points[i] = self.webPoints[i]
            self.set_param_values()
        

def main():
    # Clean up the terminal
    system('clear')
    print("#"*70)
    # Write the main top title with the name of the node
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    # Wait for one second, just to make sure roscore is running first
    time.sleep(1)
    # Start the node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    # Create the object transformations
    transformations = Transformations()
    # Set the names of the topic
    transformations.subTopicHumanPoseName = TOPIC_S1_POSE
    # Start publishers and subscribers
    transformations.start_subscribers()
    transformations.start_publishers()

    # Main loop that is running while roscore is up
    while not rospy.is_shutdown():
        # run the main function
        transformations.transformations()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass