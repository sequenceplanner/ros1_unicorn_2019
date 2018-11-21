#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Husquarna Research Platform Pose Updater
    # V.0.0.1.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import sys
import os
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ros1_unicorn_2019.msg import UpdaterSPToUni
from ros1_unicorn_2019.msg import UpdaterUniToSP

class hrp_pose_updater():

    def __init__(self):

        rospy.init_node('ur_pose_updater', anonymous=False)
      
        rospy.Subscriber("/unicorn_roscontrol/updater_sp_to_uni", UpdaterSPToUni, self.sp_callback)
        rospy.Subscriber("/robot1/odom", Odometry, self.robot1OdomCallback)
        rospy.Subscriber("/robot2/odom", Odometry, self.robot2OdomCallback)
        self.pose_lists_publisher = rospy.Publisher("unicorn_roscontrol/updater_uni_to_sp", UpdaterUniToSP, queue_size=10)

        self.rospack = rospkg.RosPack()

        self.input = self.rospack.get_path('ros1_unicorn_2019') + '/pose_lists/hrp_poses.csv'
        self.oldpose = self.rospack.get_path('ros1_unicorn_2019') + '/pose_lists/_hrp_oldpose.csv'
        self.newpose = self.rospack.get_path('ros1_unicorn_2019') + '/pose_lists/_hrp_newpose.csv'
        self.pose_name = ''
        self.prev_pose_name = ''
        self.pose_type = ''
        self.robot1_pose = []
        self.robot2_pose = []
        
        self.rate = rospy.Rate(10)
       
        rospy.sleep(5)

        self.main()


    def main(self):

        current_pose_list = UpdaterUniToSP()

        while not rospy.is_shutdown():
            self.read_and_publish_pose_list()
            current_pose_list.pose_list = self.pl
            self.pose_lists_publisher.publish(current_pose_list)
            self.rate.sleep()
            pass
        
        rospy.spin()


    def read_and_publish_pose_list(self):
        pose_list = []
        with open(self.input, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            self.pl = pose_list
        

    def update_pose_split(self, name, pose):
        with open(self.input, "rb") as f_in, open(self.oldpose, "wb") as f_op, open(self.newpose, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_pose_merge()
            

    def update_pose_merge(self):
        with open(self.newpose, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(self.oldpose, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(self.input)
        os.remove(self.newpose)
        os.rename(self.oldpose, self.input)


    def append_new_pose(self, name, pose):
        with open(self.input, 'a') as csv_write:
            csv_writer = csv.writer(csv_write, delimiter=':')
            csv_writer.writerow([name, pose])


    def robot1OdomCallback(self, data):

        self.robot1_pose = [data.pose.pose.position.x, 
                            data.pose.pose.position.y, 
                            data.pose.pose.position.z, 
                            data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y,
                            data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]
    
    
    def robot2OdomCallback(self, data):

        self.robot2_pose = [data.pose.pose.position.x, 
                            data.pose.pose.position.y, 
                            data.pose.pose.position.z, 
                            data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y,
                            data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]

    
    def sp_callback(self, data):
        self.robot = data.robot
        self.pose_name = data.pose_name

        print(self.pose_name)

        if self.pose_name == "reset":
            self.prev_pose_name = "reset"
        else:
            pass

        if self.robot == "robot1":
            if self.pose_name != self.prev_pose_name:
                with open(self.input, 'r') as csv_read:
                    csv_reader = csv.reader(csv_read, delimiter=':')
                    if all((row[0] != self.pose_name) for row in csv_reader):
                        self.append_new_pose(self.pose_name, self.robot1_pose)
                    else:
                        self.update_pose_split(self.pose_name, self.robot1_pose)

            else:
                pass

        if self.robot == "robot2":
            if self.pose_name != self.prev_pose_name:
                with open(self.input, 'r') as csv_read:
                    csv_reader = csv.reader(csv_read, delimiter=':')
                    if all((row[0] != self.pose_name) for row in csv_reader):
                        self.append_new_pose(self.pose_name, self.robot2_pose)
                    else:
                        self.update_pose_split(self.pose_name, self.robot2_pose)

            else:
                pass


if __name__ == '__main__':
    try:
        hrp_pose_updater()
    except rospy.ROSInterruptException:
        pass
