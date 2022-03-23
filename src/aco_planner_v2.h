//
// Created by fabio on 27/12/21.
//

#ifndef ANT_COLONY_PATH_PLANNER_ACO_PLANNER_V2_H
#define ANT_COLONY_PATH_PLANNER_ACO_PLANNER_V2_H

/***** ACO Dependencies *****/
#include "ACO_v2.h"
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

namespace aco_ros{
    class aco_planner_v2 : public nav_core::BaseGlobalPlanner{
    public:
        aco_planner_v2();
        ~aco_planner_v2();
        aco_planner_v2(string name, costmap_2d::Costmap2DROS* costmap_ros);

        /***** GLOBAL PLANNER EXTENSION *****/
        void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        /***** UTILS *****/
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);

        /***** ACO MATRIX MANAGEMENT *****/
        void initializePheromoneMatrix(OccupancyGridMap* map, double initialPheromoneValue);
        void initializeTotalMatrix(OccupancyGridMap* map, double alpha, double beta);

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

        vector<vector<double>> pheromoneMatrix_; // Pheromone matrix.
        vector<vector<double>> total_; // Pheromone + Heuristic matrix
        vector<Ant*> colony_; // Ant colony

        ACO_v2* planner;

        ros::Publisher planPublisher;
    };
}



#endif //ANT_COLONY_PATH_PLANNER_ACO_PLANNER_V2_H
