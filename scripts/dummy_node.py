#!/usr/bin/env python

# *****************************************************************************
#                                                                             *
#       Copyright (c) 2020  Nuctech Netherlands BV                            *
#                                                                             *
#   All rights reserved. Nuctech Netherlands source code is an unpublished    *
#   work and the use of a copyright notice does not imply otherwise.          *
#   This source code contains confidential, trade secret material             *
#   of Nuctech Netherlands. Any attempt or participation in deciphering,      *
#   decoding, reverse engineering or in any way altering the source           *
#   code is strictly prohibited, unless the prior written consent of          *
#   Nuctech Netherlands is obtained. This is proprietary and confidential     *
#   to Nuctech Netherlands.                                                   *
#                                                                             *
# *****************************************************************************

import rospy

if __name__ == '__main__':
    rospy.init_node("dummy")

    rospy.loginfo("{} node started...".format(rospy.get_name()))
    
    # rospy.sleep(1)
    
    rate = rospy.Rate(10) # 10Hz rate

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()