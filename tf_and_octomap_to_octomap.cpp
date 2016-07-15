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

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/master.h>

#include <iostream>
#include <cstdlib>

Eigen::Affine3d offset;
boost::shared_ptr<iDynUtils> idynutils;
octomap_msgs::OctomapWithPose w_Octomap;

double inf = std::numeric_limits<double>::infinity();
octomath::Vector3 min(-inf, -inf, -inf);
octomath::Vector3 max(inf, inf, inf);

void callbackShape(const shape_msgs::SolidPrimitiveConstPtr& msg)
{
    if(msg->type == 0 && msg->dimensions.size() == 6) // custom msg
    {
        min.x() = msg->dimensions[0]; min.y() = msg->dimensions[1]; min.z() = msg->dimensions[2];
        max.x() = msg->dimensions[3]; max.y() = msg->dimensions[4]; max.z() = msg->dimensions[5];
        std::cout << "Filtering octomap from:\n"
                  << min << "\nto\n" << max << std::endl;
    } else std::cout << "Unknown shape received" << std::endl;
}

void callbackPose(const geometry_msgs::PoseConstPtr& msg)
{
    tf::poseMsgToEigen(*msg, offset);
    std::cout << "Pose received:\n" << offset.affine() << std::endl;
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

bool loadBag(std::string bag_file,
             octomap_msgs::Octomap::ConstPtr& octomapMsg,
             tf2_msgs::TFMessage::ConstPtr& tfMsg,
             geometry_msgs::TransformStamped& ts,
             bool tf_optional = true)
{
    rosbag::Bag bag;
    bag.open(std::string(PROJ_DATA_DIR) + bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/octomap_binary"));
    topics.push_back(std::string("/tf"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool octomap_loaded = false;
    bool tf_loaded = false;
    bool tf_found = false;

    for(rosbag::View::const_iterator i_m = view.begin(); i_m != view.end(); ++i_m)
    {
        std::cout << ".";
        rosbag::MessageInstance m = *i_m;
        if(m.getTopic() == "/octomap_binary")
        {
            std::cout << "..reading octomap binary..";
            octomap_msgs::Octomap::ConstPtr msgOctoMap = m.instantiate<octomap_msgs::Octomap>();
            if(msgOctoMap != NULL)
            {
                std::cout << "octomap updated" << std::endl;
                octomap_loaded = true;
                octomapMsg = msgOctoMap;
            }
        }

        if(m.getTopic() == "/tf")
        {
            std::cout << "..reading tf..";
            tf2_msgs::TFMessage::ConstPtr msgTf = m.instantiate<tf2_msgs::TFMessage>();
            if(msgTf != NULL)
            {
                std::cout << "tf updated" << std::endl;
                tf_loaded = true;
                tfMsg = msgTf;
            }
        }

        if(octomap_loaded && tf_loaded)
        {
            for (unsigned int i = 0; i < tfMsg->transforms.size(); ++i)
            {
                if (tfMsg->transforms[i].child_frame_id == octomapMsg->header.frame_id)
                {
                    tf_found = true;
                    ts = tfMsg->transforms[i];
                    std::cout << "Found tf from "
                              << tfMsg->transforms[i].header.frame_id
                              << " to "
                              << tfMsg->transforms[i].child_frame_id << std::endl;
                }
            }
        }

        if(octomap_loaded && (tf_loaded && tf_found))
            break;
    }

    if(!(octomap_loaded))
    {
        std::cout << "could not find octomap msg";
        if(!tf_found && !tf_optional)
            std::cout << " and tf ";
        std::cout << "in the specified bag file" << std::endl;
        return false;
    } else if(!tf_found && !tf_optional) {
            std::cout << "could not find tf in the specified bag file" << std::endl;
        return false;
    }

    if(octomapMsg != NULL)
        std::cout << "octomap loaded succesfully" << std::endl;
    else return false;
    if(tfMsg != NULL)
        std::cout << "tf loaded succesfully" << std::endl;
    else if(!tf_optional) return false;
    bag.close();

    return true;
}

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cout << "usage: tf_and_octomap_to_octomap rosbag.bag" << std::endl;
        std::cout << "notice that rosbag.bag should lie in the data/ folder" << std::endl;
        return 0;
    }

    ros::init(argc, argv, "octomap_publisher");
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

    ros::Publisher  octomap_publisher = node_handle->advertise<octomap_msgs::Octomap>("/octomap_binary", 1);
    ros::Publisher  planning_scene_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    ros::Publisher  display_robot_state_publisher = node_handle->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
    ros::Subscriber pose_subscriber =  node_handle->subscribe("/poser",  1, callbackPose);
    ros::Subscriber shape_subscriber = node_handle->subscribe("/shaper", 1, callbackShape);
    std::string urdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.urdf";
    std::string srdf_file = std::string(PROJ_DATA_DIR) + "bigman/bigman.srdf";
    std::cout << "OPENING ROS BAG:\n" + std::string(PROJ_DATA_DIR) + argv[1] + "\n..";

    octomap_msgs::Octomap::ConstPtr octomapMsg;
    tf2_msgs::TFMessage::ConstPtr tfMsg;
    geometry_msgs::TransformStamped ts;
    Eigen::Affine3d from_T_to, from_T_to2, w_T_from, w_T_to2, to2_T_to;
    std::string pose_from, pose_to;

    if(loadBag(argv[1], octomapMsg, tfMsg, ts))
    {
        if(tfMsg != NULL)
        {
            geometry_msgs::Transform t = ts.transform;
            pose_to = octomapMsg->header.frame_id;
            pose_from = ts.header.frame_id;
            tf::transformMsgToEigen(t, from_T_to);
        }
    } else return -1;


    idynutils.reset(new iDynUtils("bigman", urdf_file, srdf_file));
    yarp::sig::Vector q = getGoodInitialPositionBigman(*idynutils);
    idynutils->updateiDyn3Model(q, true);
    idynutils->updateRobotState();

    to2_T_to.setIdentity();
    if(tfMsg != NULL)
    {
        w_T_from = idynutils->moveit_planning_scene->getCurrentState().getFrameTransform(pose_from);
        w_T_to2 = idynutils->moveit_planning_scene->getCurrentState().getFrameTransform(pose_to);
        from_T_to2 = w_T_from.inverse() * w_T_to2;
        to2_T_to = from_T_to2.inverse() * from_T_to;
    }

    geometry_msgs::Pose to2_T_to_pose; tf::poseEigenToMsg(to2_T_to, to2_T_to_pose);


    w_Octomap.octomap = *octomapMsg;
    w_Octomap.header.frame_id = octomapMsg->header.frame_id;
    w_Octomap.origin = to2_T_to_pose;
    offset.setIdentity();

    std::cout << " DONE" << std::endl;
    std::cout.flush();

    ros::Rate r(10);
    while(!ros::isShuttingDown())
    {
        ros::spinOnce();

        // void updateAndPublish()
        {
            Eigen::Affine3d origin; tf::poseMsgToEigen(w_Octomap.origin, origin);
            tf::poseEigenToMsg(origin*offset, w_Octomap.origin);
            idynutils->updateOccupancyMap(w_Octomap);
            octomap_msgs::Octomap octomap_msg = idynutils->getPlanningSceneMsg().world.octomap.octomap;
            octomap_msg.header.frame_id = idynutils->getPlanningSceneMsg().world.octomap.header.frame_id;
            if(octomap_msg.header.frame_id == "") octomap_msg.header.frame_id = "/world";
            octomap_publisher.publish(idynutils->getPlanningSceneMsg().world.octomap.octomap);
            planning_scene_publisher.publish(idynutils->getPlanningSceneMsg());
            display_robot_state_publisher.publish(idynutils->getDisplayRobotStateMsg());
        }

        r.sleep();
    }
}
