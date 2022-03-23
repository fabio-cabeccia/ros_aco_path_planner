//
// Created by fabio on 14/09/2021.
//

#ifndef ANT_COLONY_PATH_PLANNER_PATH_H
#define ANT_COLONY_PATH_PLANNER_PATH_H


/***** ROS LIBRARIES *****/
#include <ros/ros.h>

#include "OccupancyGrid.h"
#include <vector>

using namespace std;

class Path {
public:
    /**
     * @brief Default constructor
     */
    Path();

    /**
     * @brief Default destructor
     */
    ~Path();

    /**
     * @brief Copy constructor
     * @param path Vector to copy
     */
    Path(OccupancyGridMap* map, vector<unsigned int> path);

    /**
     * @brief Standard getter/setter methods
     */
    vector<unsigned int> getPath();

    double getCost();

    void setPath(vector<unsigned int> path);

    void setCost(double cost);

    /**
     * @brief Insert a cell in the path.
     * @param map
     * @param index
     * @param cellID
     */
    void insertCell(OccupancyGridMap* map, unsigned int cellID);

    double computeCost(OccupancyGridMap* map);

    /**
     * @brief Verify that the path can be taken
     * @param map
     * @return True if it is obstacle free and feasible
     */
    bool feasible(OccupancyGridMap* map);

    void clearPath();

    void popBack();


private:
    vector<unsigned int> path_; // Path data structure. Every element is the index of the correspondent cell in the map.
    double cost_; // Path cost.

};


#endif //ANT_COLONY_PATH_PLANNER_PATH_H
