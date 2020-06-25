#! /usr/bin/env python

import rospy
import time
from std_srvs.srv import Empty
from common import setup_sim


def main():
    rospy.init_node("data_generation_node")
    setup_sim()


if __name__ == "__main__":
    main()
