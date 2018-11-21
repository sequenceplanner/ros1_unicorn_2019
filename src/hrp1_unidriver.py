#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Unicorn Driver for the Husquarna Reseach Platform
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import numpy
import ast
import sys
import os
import csv
import time
from std_msgs.msg import Header
from moveit_commander import PlanningSceneInterface as psi
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros1_unicorn_2019.msg import HrpSPToUni
from ros1_unicorn_2019.msg import HrpUniToSP

class hrp1_unidriver():

    def __init__(self):

        rospy.init_node('hrp_unidriver', anonymous=False)
        self.scene = psi("world")

        rospy.Subscriber("/unicorn_roscontrol/robot1/hrp_sp_to_uni", HrpSPToUni, self.spCallback)
        rospy.Subscriber("/robot1/odom", Odometry, self.odomCallback)
        self.statePub = rospy.Publisher('/unicorn_roscontrol/robot1/hrp_uni_to_sp', HrpUniToSP, queue_size=10)
        self.goalPub = rospy.Publisher('/robot1/goal', PoseStamped, queue_size=10)

        self.rospack = rospkg.RosPack()
        self.pose_file = self.rospack.get_path('ros1_unicorn_2019') + '/pose_lists/hrp_poses.csv'
    
        self.ref_pos = ''
        self.prev_ref_pos = ''
        self.act_pos = ''
        self.isclose_tolerance = 0.05
       
        rospy.sleep(8)
        self.rate = rospy.Rate(10)

        asdf_pose = [10,10,0,0,0,0,0]

        car_pose = Pose()

        car_pose.position.x = asdf_pose[0]
        car_pose.position.y = asdf_pose[1]
        car_pose.position.z = asdf_pose[2]
        car_pose.orientation.x = asdf_pose[3]
        car_pose.orientation.y = asdf_pose[4]
        car_pose.orientation.z = asdf_pose[5]
        car_pose.orientation.w = asdf_pose[6]


        #def add_mesh(self, name, pose, filename, size = (1, 1, 1)):
        self.scene.add_mesh("McqueenCar", car_pose, "McqueenCar.STL", size = (1, 1, 1))  

        self.main()


    def main(self):

        self.msg = HrpUniToSP()    

        while not rospy.is_shutdown():
            
            with open(self.pose_file, 'r') as f_in:
                csv_reader = csv.reader(f_in, delimiter=':')
                for row in csv_reader:
                    pos = ast.literal_eval(row[1])
                    if all(numpy.isclose(self.robot_pose[i], pos[i], atol=self.isclose_tolerance) for i in range(0, 6)):
                        self.act_pos = row[0]
                        break
                    else:
                        self.act_pos = "unknown"
                        pass

            self.msg.act_pos = self.act_pos
            self.msg.got_ref_pos = self.ref_pos
            self.statePub.publish(self.msg)
            self.rate.sleep()
        
        rospy.spin()


    def move(self, ref_pos):

        goal = PoseStamped()

        with open(self.pose_file, 'r') as f_in:
            csv_reader = csv.reader(f_in, delimiter=':')
            for row in csv_reader:
                if row[0] == ref_pos:
                    saved_pose = ast.literal_eval(row[1])
                    
                    goal.pose.position.x = saved_pose[0]
                    goal.pose.position.y = saved_pose[1]
                    goal.pose.position.z = saved_pose[2]
                    goal.pose.orientation.x = saved_pose[3]
                    goal.pose.orientation.y = saved_pose[4]
                    goal.pose.orientation.z = saved_pose[5]
                    goal.pose.orientation.w = saved_pose[6]

                    self.goalPub.publish(goal)
                    rospy.sleep(1)
                else:
                    pass


    def spCallback(self, data):
        
        self.ref_pos = data.ref_pos

        if self.ref_pos == "reset":
            self.prev_ref_pos = "reset"
        else:
            pass

        if self.ref_pos != self.prev_ref_pos:
            self.prev_ref_pos = self.ref_pos
            self.move(self.ref_pos)
        else:
            pass


    def odomCallback(self, data):

        self.robot_pose = [data.pose.pose.position.x, 
                            data.pose.pose.position.y, 
                            data.pose.pose.position.z, 
                            data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y,
                            data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]


if __name__ == '__main__':
    try:
        hrp1_unidriver()
    except rospy.ROSInterruptException:
        pass
