#!/usr/bin/env python3

# Importar modulos
import rospy
from rospy.exceptions import ROSException
from reemc_pose_imitation.msg import human_pose
import time
import os
import numpy as np
import json
# Parametros y variables

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "transformations_pose"
TOPIC_S1_POSE = PACKAGE_NAME+"/pose_human"

# Class Agent


class Transformations:
    """Transformations class"""

    def __init__(self):
        self._subTopicHumanPoseName = None
        self.dataHumanPose = human_pose()
        self.paramPoints = []
        self.paramJointNames = []
        self.points = []

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
        rospy.Subscriber(self._subTopicHumanPoseName,
                        human_pose, self.data_pose_callback)

    def start_publishers(self) -> None:
        pass

    def get_param_values(self) -> None:
        self.paramJointNames = rospy.get_param('/play_motion/motions/my_move/joints')
        self.paramPoints = rospy.get_param('/play_motion/motions/my_move/points')
        Points = list(self.paramPoints[0].values())
        self.points = Points[0]

    def set_param_values(self) -> None:
        AuxparamPoints = self.paramPoints[0]
        AuxparamPoints['positions'] = self.points
        self.paramPoints[0] = AuxparamPoints
        rospy.set_param('/play_motion/motions/my_move/points',self.paramPoints)

    def transformations(self) -> None:
        # Points to be modified: 
        self.get_param_values()
        self.points[0] = 1.3
        self.points[1] = 0.8 
        self.set_param_values() 
        print(self.paramJointNames)
        time.sleep(1)
        print("Modified: ", self.paramPoints)
        

def main():
    os.system('clear')
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    time.sleep(1)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    """Inicializar el objeto Transformations"""
    objNode = Transformations()
    objNode.subTopicHumanPoseName = TOPIC_S1_POSE
    objNode.start_subscribers()
    objNode.start_publishers()
    while not rospy.is_shutdown():
        objNode.transformations()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass