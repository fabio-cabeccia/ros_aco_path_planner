//
// Created by fabio on 15/09/2021.
//

#ifndef ANT_COLONY_PATH_PLANNER_ACO_ROS_PLANNER_H
#define ANT_COLONY_PATH_PLANNER_ACO_ROS_PLANNER_H

/***** ACO Dependencies *****/
#include "ACO.h"
#include "Path.h"
#include "Ant.h"
#include "OccupancyGrid.h"
/***** ROS dependencies *****/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
/***** C++ libraries *****/
/** include standard libraries **/
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>

using namespace std;
namespace aco_ros {
    class aco_ros_planner : public nav_core::BaseGlobalPlanner {
    public:
        aco_ros_planner();
        ~aco_ros_planner();

        aco_ros_planner(string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        int convertToCellIndex(float x, float y);

        int getCellIndex(float x, float y);

        bool isValid(float x, float y);

        void convertToCoordinates(int index, float &x, float &y);

        int getCellRowID(int index);
        int getCellColID(int index);

        void convertCoordinates(float x, float y);

        bool worldToMap(double wx, double wy, double& mx, double& my);

        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);

        void updateMap();

    private:

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        string name_;
        string frame_id_;
        bool initialized_;

        unsigned int originX;
        unsigned int originY;

        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        OccupancyGridMap* occupancyGridMap;

        ACO* planner;

        ros::Publisher planPublisher;
    };
}

#endif //ANT_COLONY_PATH_PLANNER_ACO_ROS_PLANNER_H
