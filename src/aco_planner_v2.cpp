//
// Created by fabio on 27/12/21.
//

#include "aco_planner_v2.h"
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(aco_ros::aco_planner_v2, nav_core::BaseGlobalPlanner)

namespace aco_ros{
    /***** CONSTRUCTORS/DESTRUCTORS *****/
    aco_planner_v2::aco_planner_v2() : costmap_(nullptr), initialized_(false){}
    aco_planner_v2::aco_planner_v2(string name, costmap_2d::Costmap2DROS *costmap_ros) {
        initialize( name, costmap_ros );
    }
    aco_planner_v2::~aco_planner_v2() = default;

    /***** GLOBAL PLANNER EXTENSION *****/
    void aco_planner_v2::initialize(string name, costmap_2d::Costmap2DROS *costmap_ros) {
        if( !initialized_ ) {
            ROS_INFO("Initializing ACO planner.");
            ofstream traceFile_ros;
            traceFile_ros.open("traceFile_ros.txt");
            ros::NodeHandle private_nh("~/" + name);
            // Initialize map
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            unsigned char *mapChar = costmap_->getCharMap();
            // Get origin
            costmap_->worldToMap(costmap_->getOriginX(), costmap_->getOriginY(), originX, originY);
            frame_id_ = costmap_ros_->getGlobalFrameID();

            // Generate an OccupancyGridMap from costmap_
            occupancyGridMap = new OccupancyGridMap();
            // Size
            unsigned int width = costmap_->getSizeInCellsX();
            unsigned int height = costmap_->getSizeInCellsY();
            double resolution = costmap_->getResolution();
            // Content
            auto **mapData = new unsigned int *[height];
            // Initialize the matrix
            for (int i = 0; i < height; i++) {
                mapData[i] = new unsigned int[width];
            }
            int cnt = 0;
            traceFile_ros << "Costmap converted into Occupancy Grid " << endl;
            for (unsigned int iy = 0; iy < height; iy++) {
                for (unsigned int ix = 0; ix < width; ix++) {
                    unsigned char cost = costmap_->getCost(iy, ix);
                    // ROS_INFO("CURR CELL COST %d", cost);
                    if (cost == 0 || cost == -1 ) {
                        //ROS_INFO("CELL (%d, %d) IS FREE AND COST %d", iy, ix, cost);
                        mapData[iy][ix] = 0;
                        traceFile_ros << mapData[iy][ix] << " ";
                        cnt++;
                    } /* else if (cost == 255) {
                        //ROS_INFO("IS UNKNOWN AND COST %d", cost);
                        mapData[iy][ix] = -1;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        //cnt++;
                    } else if (cost > 190) {
                        //ROS_INFO("IS LETHAL AND COST %d", cost);
                        mapData[iy][ix] = 100;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        cnt++;
                    } else if (cost > 150){
                        ROS_INFO("IS OCCUPIED AND COST %d", cost);
                        mapData[iy][ix] = 99;
                        traceFile_ros <<  mapData[iy][ix] << " ";
                        //cnt++;
                    }
                    else {
                        ROS_INFO("HAS THIS COST %d", cost);
                        mapData[iy][ix] = cost;//char(1 + (97 * (cost - 1)) / 251);
                        traceFile_ros << mapData[iy][ix] << " ";
                        //cnt++;
                    } */
                    else if( cost >= 128 ){
                        mapData[iy][ix] = 100;
                    }
                    else{
                        mapData[iy][ix] = cost;
                    }
                }
                traceFile_ros << endl;
            }
            // Update the map
            occupancyGridMap->setWidth(width);
            occupancyGridMap->setHeight(height);
            occupancyGridMap->setResolution(resolution);
            occupancyGridMap->setMapLayout(mapData);
            ROS_INFO("Free cells: %d", cnt);
            ROS_INFO("HEIGHT %u , WIDTH %u, RESOLUTION %.1f", height, width, resolution);
            ROS_INFO("Costmap origin (%d , %d) and cost %d", originX, originY,
                     occupancyGridMap->getCellCost(originX, originY));

            // ACO planner's parameters
            double alpha = 1; // Pheromone information weight.
            double beta = 5; // Heuristic information weight.
            double evaporationRate = 0.5; // Pheromone evaporation rate.
            double initialPheromoneValue = 50; // Initialization value.
            int antNumber = 20;
            int iterationsLimit = 10;
            long int seed = 12345;
            // Prepare pheromone and total matrix
            initializePheromoneMatrix(occupancyGridMap, initialPheromoneValue);
            initializeTotalMatrix(occupancyGridMap, alpha, beta);
            // Generate visited vector for that will be used by all colony's component that uses row major indexing
            vector<bool> visited(height * width, false);
            // Initialize the colony
            //vector<Ant* > initCol(antNumber, new Ant());
            //colony_ = initCol;
            colony_.reserve(antNumber);
            for (int i = 0; i < antNumber; ++i){
                colony_.emplace_back(new Ant());
                colony_[i]->setName(i+1);
                colony_[i]->setVisited(visited);
            }
            ROS_INFO("Colony initialized. Size %zu", colony_.size());
            // Compute total information
            // Generate the planner
            planner = new ACO_v2(alpha, beta, evaporationRate, antNumber, iterationsLimit, seed, colony_,
                                 pheromoneMatrix_, total_);
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
    bool aco_planner_v2::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
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
        unsigned int startX, startY, goalX, goalY;
        // Check if the start position is inside the costmap
        if (!costmap_->worldToMap(wx, wy, startX_i, startY_i)) {
            ROS_WARN(
                    "The robot's start position is off the global costmap.");
            return false;
        }

        // Check if goal position is inside the costmap
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        if (!costmap_->worldToMap(wx, wy, goalX_i, goalY_i)) {
            ROS_WARN_THROTTLE(1.0,
                              "The goal sent to the ACO planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        // Clear the starting cell within the occupancy grid because we know it can't be an obstacle
        clearRobotCell(start, startX_i, startY_i);


        // Transform start and goal from pose stamped to cellIndex if they are valid
        unsigned int startCellIndex = costmap_->getIndex(startX_i, startY_i);
        unsigned int goalCellIndex = costmap_->getIndex(goalX_i, goalY_i);
        //ROS_INFO("Start cell = (%d, %d), Start index = %d Goal cell = (%d, %d), Goal index = %d", startX_i, startY_i, startCellIndex, goalX_i, goalY_i, goalCellIndex);
        //planner->computeTotalInformation(occupancyGridMap, goalCellIndex);
        vector<unsigned int> path_; // Path found by ACO
        ros::Time sTime = ros::Time::now();
        path_ = planner->computePaths(occupancyGridMap, startCellIndex, goalCellIndex);
        ros::Duration eTime = ros::Time::now() - sTime;
        // path = planner->computeParallelPaths(occupancyGridMap, startCellIndex, goalCellIndex);

        if (!path_.empty()) {
            ROS_INFO("ACO found a path in %f seconds!", eTime.toSec());
            // Prepare the plan
            plan.push_back(start);
            ros::Time plan_time = ros::Time::now();
            // convert the points to poses
            for (unsigned int i : path_) {
                //std::cout << path[i].first << " " << path[i].second << std::endl;
                unsigned int x_i = 0;
                unsigned int y_i = 0;
                wx = 0.0;
                wy = 0.0;
                // convertToCoordinates(path->getPath()[i], x, y);
                costmap_->indexToCells(i, x_i, y_i);
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
            //publishGlobalPlan(plan);
            return true;
        }
        ROS_ERROR("ACO planner couldn't find a path.");
        return false;
    }

    /***** UTILS *****/
    void aco_planner_v2::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        occupancyGridMap->setFree(mx, my);
        // setCost(mx, my, costmap_2d::FREE_SPACE);
    }
    void aco_planner_v2::publishGlobalPlan(vector<geometry_msgs::PoseStamped> &path) {
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

    /***** ACO MATRIX MANAGEMENT *****/
    void aco_planner_v2::initializePheromoneMatrix(OccupancyGridMap* map, double initialPheromoneValue) {
        ROS_INFO("Initializing pheromone matrix.");
        //traceFile << "Initial pheromone matrix" << endl ;
        vector<vector<double>> init(map->getHeight(), vector<double>(map->getWidth()));
        pheromoneMatrix_ = init;

        for (int i = 0; i < map->getHeight(); i++) {

            for (int j = 0; j < map->getWidth(); j++) {

                /* if (map->isFree(i, j) ) {
                    pheromoneMatrix_[i][j] = initialPheromoneValue;
                    //traceFile << getInitialPheromoneValue() << " ";
                }
                else if ( map->isValid(i, j) ){
                    pheromoneMatrix_[i][j] = 0.0;
                } */
                pheromoneMatrix_[i][j] = initialPheromoneValue;
            }
            //traceFile << endl;
        }
    }
    void aco_planner_v2::initializeTotalMatrix(OccupancyGridMap* map, double alpha, double beta) {
        ROS_INFO("Initializing pheromone + heuristic matrix.");
        vector<vector<double>> init( map->getHeight(), vector<double>(map->getWidth()) );
        total_ = init;
        double heuristic = 0;
        for (unsigned int i = 0; i < map->getHeight(); i++) {
            for (unsigned int j = 0; j < map->getWidth(); j++) {
                // The heuristic information is the inverse of the cell cost
                if ( map->isFree(i,j) )
                    heuristic = 1.0;
                else
                    heuristic = 1.0/map->getCellCost(i, j);

                total_[i][j] = pow(pheromoneMatrix_[i][j], alpha) * pow(heuristic, beta);
            }
        }
    }
}

double euclideanDistance(unsigned int x1, unsigned int y1,unsigned int x2, unsigned int y2){
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}
