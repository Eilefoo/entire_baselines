#! /usr/bin/env python

import rospy
import time
from std_srvs.srv import Empty


rospy.init_node("data_generation_node")

print("Waiting for service: /gazebo/reset_world ")
rospy.wait_for_service("/gazebo/reset_world")
reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

for i in range(1,20):
    reset_world()
    time.sleep(2)

# if __name__ == "__main__":
#         print("Testing the World Reset")


    