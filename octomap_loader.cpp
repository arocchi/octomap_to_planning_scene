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

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/master.h>

#include <iostream>
#include <cstdlib>

Eigen::Affine3d pose;
Eigen::Affine3d offset;
boost::shared_ptr<iDynUtils> idynutils;
ros::Publisher octomap_publisher;

octomath::Pose6D poseEigenToOctomap(Eigen::Affine3d transform)
{
        Eigen::Quaterniond qd(transform.rotation());
        octomath::Quaternion q(qd.w(), qd.x(), qd.y(), qd.z());
        octomath::Vector3 v(transform.translation().x(),
                            transform.translation().y(),
                            transform.translation().z());
        octomath::Pose6D t(v,q);
        return t;
        
        /* can be done also as:
           tf::Pose tf;
           tf::poseEigenToTF(transform, tf)
           return tf::poseTfToOctomap(tf);
        */
}

// first transform, then filter (NOTE: we should do the same also on the original octomap
octomap_msgs::Octomap transformAndFilterOctomap(Eigen::Affine3d transform, 
                               const octomap_msgs::Octomap& octomap_msg,
                               octomath::Vector3 min, octomath::Vector3 max)
{
    octomap_msgs::Octomap msg;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    
    octomap::OcTree* newOctree = new octomap::OcTree(octomap_msg.resolution);
    
    octomath::Pose6D t = poseEigenToOctomap(transform);
    
    /*octree->toMaxLikelihood();
    octree->prune();
    octree->expand();*/
    std::cout << "starting octmap transform ...";
     for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
       end=octree->end_leafs(); it!= end; ++it)
    {
        //manipulate node, e.g.:
        octomath::Vector3 newCoord = t.transform(it.getCoordinate());
        if(newCoord.x() > min.x() &&
           newCoord.x() < max.x() &&
           newCoord.y() > min.y() &&
           newCoord.y() < max.y() &&
           newCoord.z() > min.z() &&
           newCoord.z() < max.z())
        newOctree->updateNode(newCoord,
                              it->getValue());
        
    }
    /*
    newOctree->toMaxLikelihood();
    newOctree->prune();
    */
    std::cout << "done" << std::endl;


    octomap_msgs::fullMapToMsg(*newOctree, msg);
    
    delete(newOctree);
    delete(octree);
    
    return msg;
}

void publishOctomap()
{
    
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

    octomap_publisher = node_handle->advertise<moveit_msgs::PlanningScene>("octomap_T", 1);

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
    
    idynutils.reset(new iDynUtils("bigman", urdf_file, srdf_file));
    yarp::sig::Vector q = getGoodInitialPositionBigman(*idynutils);
    idynutils->updateiDyn3Model(q, true);
    idynutils->updateRobotState();

    pose.setIdentity();
    geometry_msgs::Pose p;
    p.position.x = 0.0;
    p.position.y = 0.0;
    p.position.z = 0.5;
    p.orientation.x = -0.622211754092;
    p.orientation.y = 0.621716467379;
    p.orientation.z = -0.336259528987;
    p.orientation.w = 0.336527408134;
    
    tf::Pose P;
    tf::poseMsgToTF(p,P);
    
    Eigen::Affine3d T;
    tf::poseTFToEigen(P, T);
    
    octomath::Pose6D t = poseEigenToOctomap(pose*offset);
    double inf = std::numeric_limits<double>::infinity();
    octomath::Vector3 min(0.5, -0.5, -inf);
    octomath::Vector3 max(1.5,  0.5,  inf);
    
    octomap_msgs::Octomap octomapFiltered =  transformAndFilterOctomap(T, *octomapMsg, min, max);
    
    std::cout << " DONE" << std::endl;
    std::cout.flush();

    ros::Rate r(10);
    while(!ros::isShuttingDown())
    {
        octomap_publisher.publish(octomapFiltered);
        r.sleep();   
    }
}