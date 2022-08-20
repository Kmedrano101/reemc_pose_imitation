#!/usr/bin/env python3

# Importar modulos
from ntpath import join
import rospy
from rospy.exceptions import ROSException
from reemc_pose_imitation.msg import human_pose
import time
import os
import numpy as np
import math
# Parametros y variables

try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "transformations_pose"
TOPIC_S1_POSE = "/pose_human"
# Class Agent


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
        rospy.Subscriber(self._subTopicHumanPoseName,
                        human_pose, self.data_pose_callback)

    def start_publishers(self) -> None:
        pass

    def get_param_values(self) -> None:
        # Obtener todos los paramatros de los archivos .yaml dentro de reem-c y reemc_pose_imitation
        self.paramJointNames = rospy.get_param('/play_motion/motions/my_move2/joints')
        self.paramPoints = rospy.get_param('/play_motion/motions/my_move2/points')
        self.imgWeight = rospy.get_param("/image_source/weight")
        self.imgHeight = rospy.get_param("/image_source/height")
        Points = list(self.paramPoints[0].values())
        self.points = Points[0]
        self.localPoints = rospy.get_param('/local_points/points')
        # Params from web UI interface
        self.webPoints = rospy.get_param('/web_points/points')
        self.mode = rospy.get_param("/operation_mode", default=0)

    def set_param_values(self) -> None:
        AuxparamPoints = self.paramPoints[0]
        AuxparamPoints['positions'] = self.points
        self.paramPoints[0] = AuxparamPoints
        rospy.set_param('/play_motion/motions/my_move2/points',self.paramPoints)
        # Intentar actualizar solo Joints de interes 

    def get_angle(self, p1, p2, p3) -> float: # input values make it as a dict 
        angle = 0
        points = []
        if self.dataHumanPose.points_pose:
            for pose in self.dataHumanPose.points_pose:
                h, w = self.imgHeight, self.imgWeight # height of img and weight of img
                cx, cy = int(pose.position.x * w), int(pose.position.y * h)
                points.append([pose.id_pos, cx, cy]) # pose.position.z
            x1, y1 = points[p1][1:]
            x2, y2 = points[p2][1:]
            x3, y3 = points[p3][1:]
            # Calculate the Angle
            angle = math.degrees(math.atan2(y3 - y2, x3 - x2) -
                                math.atan2(y1 - y2, x1 - x2))
            if angle < 0:
                angle += 360
            self.posePoints = points
        #print(angle)
        return angle 

    def map_value(self,value,out_min,out_max,in_min,in_max) -> float: # input map values min max make it in a dict of Joints [joint_1: [0.0,2.0,0.0,120.0]]
        #value = float((value - robot_min) * (angle_max - angle_min) / (robot_max - robot_min) + angle_min)
        value = float((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
        #print(value)
        return value

    def get_distance(self,x1,x2,y1,y2):
        distance = np.sqrt((x2-x1)**2+(y2-y1)**2)
        return distance

    def transformations(self) -> None:
        # Points to be modified: 
        self.get_param_values()
        # Obtener los angulos para los Joints del Robot
        if self.mode == 0:
            if self.get_angle(23,11,13):
                # Joints Arm Left
                angle_alfa1 = self.get_angle(13,11,23)
                angle_beta1 = self.get_angle(15,11,23)
                angle_gama1 = self.get_angle(15,13,11)
                self.localPoints[self.paramJointNames.index('arm_left_2_joint')] = round(self.map_value(angle_alfa1,0.0,2.0,0,120),2)
                if angle_beta1 > angle_alfa1:
                    self.localPoints[self.paramJointNames.index('arm_left_3_joint')] = 1.5
                else:
                    self.localPoints[self.paramJointNames.index('arm_left_3_joint')] = -1.5

                self.localPoints[self.paramJointNames.index('arm_left_4_joint')] = round(self.map_value(abs(185-angle_gama1),0.0,2.2,0.0,160.0),2)
                # Joints Arm Right
                angle_alfa2 = self.get_angle(24, 12, 14)
                angle_beta2 = self.get_angle(24, 12, 16)
                angle_gama2 = self.get_angle(12, 14, 16)
                self.localPoints[self.paramJointNames.index('arm_right_2_joint')] = round(self.map_value(angle_alfa2,0.0,2.0,0,120),2)
                if angle_beta2 > angle_alfa2:
                    self.localPoints[self.paramJointNames.index('arm_right_3_joint')] = 1.5
                else:
                    self.localPoints[self.paramJointNames.index('arm_right_3_joint')] = -1.5

                self.localPoints[self.paramJointNames.index('arm_right_4_joint')] = round(self.map_value(abs(185-angle_gama2),0.0,2.2,0.0,160.0),2) 
                # Joints Head 
                d1 = self.get_distance(self.posePoints[0][1],self.posePoints[7][1],self.posePoints[0][2],self.posePoints[7][2])
                d2 = self.get_distance(self.posePoints[0][1],self.posePoints[8][1],self.posePoints[0][2],self.posePoints[8][2])
                print("distancia 1: ",d1)
                print("distancia 2: ",d2)
                if d1 > d2:
                    # Validar valor > Limites minimos
                    if abs(d1-d2) < d1*0.58:
                        value1 = d1*0.58
                    else:
                        value1 = abs(d1-d2)
                    self.localPoints[self.paramJointNames.index('head_1_joint')] = -round(self.map_value(value1,0.0,1.6,0,(d1*0.58)),2)
                else:
                    # Validar valor > Limites minimos
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
            #print(self.paramJointNames)
            #print(self.posePoints)
            #print("2do Angulo: ",self.findAngle(13, 11, 23))
            time.sleep(1)
        #print("Modified: ", self.paramPoints)
        

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