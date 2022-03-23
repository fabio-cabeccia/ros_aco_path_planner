//
// Created by fabio on 09/09/2021.
//

#include "Path.h"

Path::Path() {
    path_.clear();
    cost_ = 0;
}

Path::~Path() = default;

Path::Path(OccupancyGridMap* map, vector<unsigned int> path) {
    setPath(path);
    setCost(computeCost(map));
}

vector<unsigned int> Path::getPath() {
    return path_;
}

double Path::getCost() {
    return cost_;
}

void Path::setPath(vector<unsigned int> path) {
    path_ = path;
}

void Path::setCost(double cost) {
    cost_ = cost;
}

void Path::insertCell(OccupancyGridMap* map, unsigned int cellID) { // If using local search may be useful to also decide the position where the
    // new cell is going to be added
    /* if( !path_.empty() ){ // If the cell is not the first of the path, then cost will be updated.
        cost_ += map->getMoveCost(path_.back(), cellID);
    }
    else{
        cost_ = 0;
    } */
    path_.emplace_back(cellID);
    computeCost(map);
}

double Path::computeCost(OccupancyGridMap* map) {
    double cost = 0.0;
    for ( int i = 0; i < path_.size(); i++ ) {
        cost += 1;//map->getMoveCost(path_[i], path_[i+1]);
    }
        return cost;

    //return -1.0;
}

bool Path::feasible(OccupancyGridMap* map) {
    for (unsigned int i: getPath()) {
        if (!map->isFree(i))
            return false;
    }
    return true;
}

void Path::clearPath() {
    while( !path_.empty() )
        path_.pop_back();
    setCost(0);
}

void Path::popBack() {
    path_.pop_back();
}

