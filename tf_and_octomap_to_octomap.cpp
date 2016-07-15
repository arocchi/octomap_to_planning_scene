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
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/master.h>

#include <iostream>
#include <cstdlib>

Eigen::Affine3d to2_T_to, offset;
boost::shared_ptr<iDynUtils> idynutils;
octomap_msgs::OctomapWithPose w_Octomap;
bool octomap_received = false;
std::string pose_from, pose_to;

double inf = std::numeric_limits<double>::infinity();
octomath::Vector3 min(-inf, -inf, -inf);
octomath::Vector3 max(inf, inf, inf);

void callbackOctomap(const octomap_msgs::OctomapConstPtr& msg)
{
    std::cout << "**** octomap received" << std::endl;
    w_Octomap.octomap = *msg;
    w_Octomap.header.frame_id = msg->header.frame_id;
    octomap_received = true;
    pose_to = msg->header.frame_id;
    pose_from = "l_sole";
}

void callbackShape(const shape_msgs::SolidPrimitiveConstPtr& msg)
{
    if(msg->type == 0 && msg->dimensions.size() == 6) // custom msg
    {
        std::cout << "*** shape received" << std::endl;
        min.x() = msg->dimensions[0]; min.y() = msg->dimensions[1]; min.z() = msg->dimensions[2];
        max.x() = msg->dimensions[3]; max.y() = msg->dimensions[4]; max.z() = msg->dimensions[5];
        std::cout << "Filtering octomap from:\n"
                  << min << "\nto\n" << max << std::endl;
    } else std::cout << "Unknown shape received" << std::endl;
}

void callbackPose(const geometry_msgs::PoseConstPtr& msg)
{
    tf::poseMsgToEigen(*msg, offset);
    std::cout << "*** pose received\n" << offset.affine() << std::endl;
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
    ros::init(argc, argv, "tf_and_octomap_to_octomap");
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

    ros::Publisher  octomap_publisher = node_handle->advertise<octomap_msgs::Octomap>("/octomap_binary_filtered", 1);
    ros::Publisher  planning_scene_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    ros::Publisher  display_robot_state_publisher = node_handle->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
    tf::TransformListener listener;
    ros::Subscriber octomap_subscriber = node_handle->subscribe("/octomap_binary",  1, callbackOctomap);
    ros::Subscriber pose_subscriber =  node_handle->subscribe("/poser",  1, callbackPose);
    ros::Subscriber shape_subscriber = node_handle->subscribe("/shaper", 1, callbackShape);
    std::string urdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.urdf";
    std::string srdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.srdf";

    tf::StampedTransform st;
    Eigen::Affine3d from_T_to, from_T_to2, w_T_from, w_T_to2;

    idynutils.reset(new iDynUtils("bigman", urdf_file, srdf_file));
    yarp::sig::Vector q = getGoodInitialPositionBigman(*idynutils);
    idynutils->updateiDyn3Model(q, true);
    idynutils->updateRobotState();

    to2_T_to.setIdentity();
    offset.setIdentity();

    ros::Rate r(10);
    while(!ros::isShuttingDown())
    {
        ros::spinOnce();

        if(octomap_received)
        {
            try
            {
                listener.waitForTransform(pose_to, pose_from, ros::Time(0), ros::Duration(1.0) );
                listener.lookupTransform(pose_to, pose_from,
                                         ros::Time(0), st);
                std::cout << "*** transform received\nfrom " << pose_from << " to " << pose_to << std::endl;
                w_T_from = idynutils->moveit_planning_scene->getCurrentState().getFrameTransform(pose_from);
                w_T_to2 = idynutils->moveit_planning_scene->getCurrentState().getFrameTransform(pose_to);
                from_T_to2 = w_T_from.inverse() * w_T_to2;
                to2_T_to = from_T_to2.inverse() * from_T_to;
            } catch(...)  {
                std::cout << "xxx could not read transform from " << pose_from << " to " << pose_to << std::endl;
            }

        }

        if(octomap_received)
        // void updateAndPublish()
        {
            tf::poseEigenToMsg(to2_T_to*offset, w_Octomap.origin);
            idynutils->updateOccupancyMap(w_Octomap);
            octomap_msgs::Octomap octomap_msg = idynutils->getPlanningSceneMsg().world.octomap.octomap;
            octomap_msg.header.frame_id = idynutils->getPlanningSceneMsg().world.octomap.header.frame_id;
            std::cout << "old octomap had frame_id="
                      << w_Octomap.header.frame_id
                      <<", publishing new octomap with frame_id="
                     << octomap_msg.header.frame_id << std::endl;
            octomap_publisher.publish(octomap_msg);
            planning_scene_publisher.publish(idynutils->getPlanningSceneMsg());
            display_robot_state_publisher.publish(idynutils->getDisplayRobotStateMsg());
        }

        r.sleep();
    }
}
