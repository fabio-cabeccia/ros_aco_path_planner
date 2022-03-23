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
    void setVisited(vector<bool> visited);
    void removeVisited ( unsigned int cellID);
    void setConstructedPath(Path *path);
    void setSearchTrue();
    void setSearchFalse();

    unsigned int getPosition();
    Path *getConstructedPath(); //return the path constructed by the ant
    vector<bool> getVisited();
    int getName();

    bool isVisited(unsigned int cellID);
    bool isSearching();

    void addToVisited(unsigned int cellID);
    void clearVisited();
private:
    Path* constructedPath_; // The path built by the single ant.
    vector<bool> visited_;
    unsigned int currPos_; // Index of the current cell where the ant is.
    int name_;
    bool search;
};


#endif //ANT_COLONY_PATH_PLANNER_ANT_H
