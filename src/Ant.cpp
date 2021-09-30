//
// Created by fabio on 09/09/2021.
//

#include "Ant.h"

Ant::Ant() {
    constructedPath_ = new Path();
    constructedPath_->setCost(0);
    visited_.clear();
    currPos_ = 0;
}

Ant::Ant(Path *path, unsigned int antPos) {
    constructedPath_ = path;
    currPos_ = antPos;
}

void Ant::setPosition(unsigned int antPos) {
    currPos_ = antPos;
}
void Ant::setName(int name){
    name_ = name;
}
void Ant::setConstructedPath(Path* path) {
    constructedPath_ = path;
}

unsigned int Ant::getPosition() {
    return currPos_;
}

Path *Ant::getConstructedPath() {
    return constructedPath_;
}

vector<unsigned int> Ant::getVisited(){
    return visited_;
}

int Ant::getName(){
    return name_;
}

void Ant::clearVisited() {
    while( !visited_.empty() )
        visited_.pop_back();
}

void Ant::addToVisited(unsigned int cellID) {
    visited_.emplace_back(cellID);
}



