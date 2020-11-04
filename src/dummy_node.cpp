//*****************************************************************************
//                                                                            *
//      Copyright (c) 2020  Nuctech Netherlands BV                            *
//                                                                            *
//  All rights reserved. Nuctech Netherlands source code is an unpublished    *
//  work and the use of a copyright notice does not imply otherwise.          *
//  This source code contains confidential, trade secret material             *
//  of Nuctech Netherlands. Any attempt or participation in deciphering,      *
//  decoding, reverse engineering or in any way altering the source           *
//  code is strictly prohibited, unless the prior written consent of          *
//  Nuctech Netherlands is obtained. This is proprietary and confidential     *
//  to Nuctech Netherlands.                                                   *
//                                                                            *
//*****************************************************************************

#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "dummy_cpp");

    ros::NodeHandle nh;

    std::string node_name = ros::this_node::getName();

    ROS_INFO("node %s has been started", node_name.c_str());

    // ros::Duration(1.0).sleep();

    ros::Rate rate(10);

    while (ros::ok()) {
        ROS_INFO("Hello");
        rate.sleep();
    }

}