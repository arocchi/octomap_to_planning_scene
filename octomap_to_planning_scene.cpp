/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>

#include <kdl/frames_io.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/master.h>

#include <iostream>
#include <cstdlib>

Eigen::Affine3d pose;
Eigen::Affine3d offset;
octomap_msgs::OctomapWithPose octomapMsgWithPose;
boost::shared_ptr<iDynUtils> idynutils;
std::string frame_id;
ros::Publisher planning_scene_publisher;

void updateOctomapAndPublish()
{
    tf::poseEigenToMsg(
        pose*offset,
        octomapMsgWithPose.origin);
    idynutils->updateOccupancyMap(octomapMsgWithPose);
    planning_scene_publisher.publish(
        idynutils->getPlanningSceneMsg());
}

void callback(const geometry_msgs::PoseConstPtr& msg)
{
    tf::poseMsgToEigen(*msg, offset);
    updateOctomapAndPublish();
    std::cout << "Pose received" << std::endl;
}

yarp::sig::Vector getGoodInitialPositionBigman(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[2] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[4] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[2] = -15.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    arm[2] = -arm[2];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);

    std::cout << "Q_initial: " << q.toString() << std::endl;
    return q;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_scene_publisher");
    bool ros_is_running = ros::master::check();
    boost::shared_ptr<ros::NodeHandle> node_handle;
    if(ros_is_running)
    {
        node_handle.reset(new ros::NodeHandle());
        std::cout << "--- roscore found" << std::endl;
    } else 
    {
        std::cout << "--- roscore not found" << std::endl;
        return 0;
    }

    ros::Subscriber pose_subscriber;

    planning_scene_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    pose_subscriber = node_handle->subscribe("poser", 1, callback);

    std::string urdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.urdf";
    std::string srdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.srdf";
    std::cout << "OPENING OCTOMAP BAG:\n" + std::string(PROJ_DATA_DIR) + "octomap.bag" + "\n..";
    rosbag::Bag bag;
    bag.open(std::string(PROJ_DATA_DIR) + "octomap.bag", rosbag::bagmode::Read);
    octomap_msgs::Octomap msg;
    std::vector<std::string> topics;
    topics.push_back(std::string("/octomap_binary"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::const_iterator i_m = view.begin();
    rosbag::MessageInstance m = *i_m;
    octomap_msgs::Octomap::ConstPtr octomapMsg = m.instantiate<octomap_msgs::Octomap>();
    bag.close();
    std::cout << " DONE" << std::endl;
    std::cout.flush();

    octomapMsgWithPose.octomap = *octomapMsg;
    octomapMsgWithPose.header = octomapMsg->header;
    frame_id = octomapMsg->header.frame_id;

    idynutils.reset(new iDynUtils("bigman", urdf_file, srdf_file));
    yarp::sig::Vector q = getGoodInitialPositionBigman(*idynutils);
    idynutils->updateiDyn3Model(q, true);
    idynutils->updateRobotState();

    Eigen::Affine3d w_T_octomap = idynutils->moveit_planning_scene->getFrameTransform(frame_id);
    pose = w_T_octomap;

    offset.setIdentity();
    updateOctomapAndPublish();

    ros::spin();
}
