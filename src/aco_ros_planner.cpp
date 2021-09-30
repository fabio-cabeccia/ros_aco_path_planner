//
// Created by fabio on 15/09/2021.
//

#include "aco_ros_planner.h"
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
using namespace std;
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(aco_ros::aco_ros_planner, nav_core::BaseGlobalPlanner)

namespace aco_ros{

    aco_ros_planner::aco_ros_planner() : costmap_(nullptr), initialized_(false){}

    aco_ros_planner::aco_ros_planner(string name, costmap_2d::Costmap2DROS *costmap_ros) {
        initialize( name, costmap_ros );
    }

    aco_ros_planner::~aco_ros_planner() = default;

    void aco_ros_planner::initialize(string name, costmap_2d::Costmap2DROS *costmap_ros) {
        if( !initialized_ ){
            ROS_INFO("Initializing ACO planner.");
            ofstream traceFile_ros;
            traceFile_ros.open("traceFile_ros.txt");
            ros::NodeHandle private_nh("~/" + name);
            // Initialize map
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            // Get origin
            costmap_->worldToMap(costmap_->getOriginX(), costmap_->getOriginY(), originX, originY);
            // originX = costmap_->getOriginX();
            // originY = costmap_->getOriginY();
            frame_id_ = costmap_ros_->getGlobalFrameID();
            ROS_INFO("Costmap origin (%d , %d) and cost %d", originX, originY,  costmap_->getCost(originX, originY) );

            // Generate an OccupancyGridMap from costmap_
            occupancyGridMap = new OccupancyGridMap();
            // Size
            unsigned int width = costmap_->getSizeInCellsX();
            unsigned int height = costmap_->getSizeInCellsY();
            double resolution = costmap_->getResolution();
            // Content
            auto** mapData = new unsigned int*[height];
            // Initialize the matrix
            for( int i = 0; i < height; i++ ){
                mapData[i] = new unsigned int[width];
            }
            int cnt = 0;
            traceFile_ros << "Costmap converted into Occupancy Grid " << endl;
            for (unsigned int iy = 0; iy < height; iy++) {
                for (unsigned int ix = 0; ix < width; ix++) {
                    unsigned int cost = costmap_->getCost(iy, ix);
                    unsigned int cost1 = costmap_ros->getCostmap()->getCost(iy, ix);
                    if( cost != 0 || cost1 != 0) ROS_INFO("CURR CELL COST %d, COST1 %d", cost, cost1);
                    if (cost == 0) {
                        //ROS_INFO("IS FREE AND COST %d", cost);
                        mapData[iy][ix] = 0;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                    } else if (cost == 255) {
                        ROS_INFO("IS UNKNOWN AND COST %d", cost);
                        mapData[iy][ix] = -1;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        cnt++;
                    } else if (cost == 254) {
                        ROS_INFO("IS OCCUPIED AND COST %d", cost);
                        mapData[iy][ix] = 100;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        cnt++;
                    } else if (cost == 253){
                        ROS_INFO("IS OCCUPIED AND COST %d", cost);
                        mapData[iy][ix] = 99;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        cnt++;
                    }
                    else {
                        ROS_INFO("HAS THIS COST %d", cost);
                        mapData[iy][ix] = cost;//char(1 + (97 * (cost - 1)) / 251);
                        traceFile_ros << mapData[iy][ix] << " ";
                        cnt++;
                    }
                }
                traceFile_ros << endl;
            }
            ROS_INFO("CNT %d", cnt);
            ROS_INFO("HEIGHT %u , WIDTH %u, RESOLUTION %.1f", height, width, resolution);
            // Update the map
            occupancyGridMap->setWidth(width);
            occupancyGridMap->setHeight(height);
            occupancyGridMap->setResolution(resolution);
            occupancyGridMap->setMapLayout(mapData);

            // ACO planner's parameters
            double alpha = 0.5; // Pheromone information weight.
            double beta = 0.5; // Heuristic information weight.
            double evaporationRate = 0.2; // Pheromone evaporation rate.
            double initialPheromoneValue = 50; // Initialization value.
            int antNumber = 5;
            int iterationsLimit = 10;
            planner = new ACO(alpha, beta, evaporationRate, initialPheromoneValue, antNumber, iterationsLimit);

            // Plan publisher
            planPublisher = private_nh.advertise<nav_msgs::Path>("plan", 1);

            ROS_INFO("ACO planner successfully initialized.");
            initialized_ = true;
            traceFile_ros.close();
        }
        else{
            ROS_WARN("ACO planner already initialized, doing nothing.");
        }

    }

    bool aco_ros_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!initialized_) {
            ROS_ERROR("The planner has not been initialized.");
            return false;
        }
        // Clear the plan
        plan.clear();

        string global_frame = frame_id_;

        // Until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
        if (goal.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
            return false;
        }
        if (start.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
            return false;
        }

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        unsigned int startX_i, startY_i, goalX_i, goalY_i;
        //double startX, startY, goalX, goalY;
        // Check if the start position is inside the costmap
        if (!costmap_->worldToMap(wx, wy, startX_i, startY_i)) {
            ROS_WARN(
                    "The robot's start position is off the global costmap.");
            return false;
        }
        // Convert the start position to map coordinates
        // worldToMap(wx, wy, startX, startY);

        // Check if goal position is inside the costmap
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        if (!costmap_->worldToMap(wx, wy, goalX_i, goalY_i)) {
            ROS_WARN_THROTTLE(1.0,
                              "The goal sent to the ACO planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }
        // Convert goal position to map coordinates
        // worldToMap(wx, wy, goalX, goalY);

        // Clear the starting cell within the occupancy grid because we know it can't be an obstacle
        clearRobotCell(start, startX_i, startY_i);

        Path *path = new Path();
        // Transform start and goal from pose stamped to cellIndex if they are valid
        unsigned int startCellIndex = costmap_->getIndex(startX_i, startY_i);
        unsigned int goalCellIndex = costmap_->getIndex(goalX_i, goalY_i);
        ROS_INFO("Start cell = %d, %d, Start index = %d Goal = %d, %d, Goal index = %d", startX_i, startY_i, startCellIndex, goalX_i, goalY_i, goalCellIndex);
        path = planner->computePaths(occupancyGridMap, startCellIndex, goalCellIndex);
        // path = planner->computeParallelPaths(occupancyGridMap, startCellIndex, goalCellIndex);

        if (!path->getPath().empty()) {
            ROS_INFO("ACO found a path!");
            // Prepare the plan
            plan.push_back(start);
            ros::Time plan_time = ros::Time::now();
            // convert the points to poses
            for (int i = 0; i < path->getPath().size(); i++) {
                //std::cout << path[i].first << " " << path[i].second << std::endl;
                unsigned int x_i = 0;
                unsigned int y_i = 0;
                wx = 0.0;
                wy = 0.0;
                // convertToCoordinates(path->getPath()[i], x, y);
                costmap_->indexToCells(path->getPath()[i], x_i, y_i);
                costmap_->mapToWorld(x_i, y_i, wx, wy);
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                plan.push_back(pose);
               // ROS_INFO("%.1f %.1f Pushed back in plan", wx, wy);
            }
            publishGlobalPlan(plan);
            updateMap();
            return true;
        }
        ROS_ERROR("ACO planner couldn't find a path.");
        updateMap();
        return false;
    }

    void aco_ros_planner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        occupancyGridMap->setFree(mx, my);
        // setCost(mx, my, costmap_2d::FREE_SPACE);
    }

    void aco_ros_planner::publishGlobalPlan(vector<geometry_msgs::PoseStamped> &path) {
        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        planPublisher.publish(gui_path);
    }

    void aco_ros_planner::updateMap(){
        unsigned int width = costmap_->getSizeInCellsX();
        unsigned int height = costmap_->getSizeInCellsY();
        double resolution = costmap_->getResolution();
        // Content
        auto** mapData = new unsigned int*[height];
        // Initialize the matrix
        for( int i = 0; i < height; i++ ){
            mapData[i] = new unsigned int[width];
        }
        for (unsigned int iy = 0; iy < height; iy++) {
            for (unsigned int ix = 0; ix < width; ix++) {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
                if (cost == 0) {
                    //ROS_INFO("Cell %d %d is free.", ix, iy);
                    mapData[iy][ix] = 0;
                } else if (cost == 255) {
                    //ROS_INFO("Status of cell %d %d is unknown.", ix, iy);
                    mapData[iy][ix] = -1;
                } else if (cost == 254) {
                    //ROS_INFO("Cell %d %d is an obstacle.", ix, iy);
                    mapData[iy][ix] = 100;
                } else if (cost == 253){
                    //ROS_INFO("Cell %d %d is an obstacle.", ix, iy);
                    mapData[iy][ix] = 99;
                }
                else
                    mapData[iy][ix] = cost;//char(1 + (97 * (cost - 1)) / 251);
            }
        }

        // Update the map
        occupancyGridMap->setWidth(width);
        occupancyGridMap->setHeight(height);
        occupancyGridMap->setResolution(resolution);
        occupancyGridMap->setMapLayout(mapData);
    }
}