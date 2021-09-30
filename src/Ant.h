//
// Created by fabio on 14/09/2021.
//

#ifndef ANT_COLONY_PATH_PLANNER_ANT_H
#define ANT_COLONY_PATH_PLANNER_ANT_H
// ROS dependencies
#include <costmap_2d/costmap_2d_ros.h>
// ACO dependencies
#include "Path.h"
// C++ dependencies
#include <vector>

using namespace std;

class Ant {
public:
    Ant();

    Ant(Path *path, unsigned int antPos);


    void setPosition(unsigned int antPos);
    void setName(int name);
    void setConstructedPath(Path *path);

    unsigned int getPosition();
    Path *getConstructedPath(); //return the path constructed by the ant
    vector<unsigned int> getVisited();
    int getName();

    void addToVisited(unsigned int cellID);
    void clearVisited();
private:
    Path* constructedPath_; // The path built by the single ant.
    vector<unsigned int> visited_;
    unsigned int currPos_; // Index of the current cell where the ant is.
    int name_;
};


#endif //ANT_COLONY_PATH_PLANNER_ANT_H
