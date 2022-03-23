//
// Created by fabio on 09/09/2021.
//

#include "Ant.h"

#include <utility>

Ant::Ant() {
    constructedPath_ = new Path();
    constructedPath_->setCost(0);
    visited_.clear();
    search = false;
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
void Ant::setVisited(vector<bool> visited){
    visited_ = visited;
}
void Ant::removeVisited ( unsigned int cellID){
    visited_[cellID] = false;
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
vector<bool> Ant::getVisited(){
    return visited_;
}
int Ant::getName(){
    return name_;
}

bool Ant::isVisited(unsigned int cellID){
    return visited_[cellID];
}

void Ant::clearVisited() {
    for( int i = 0; i < visited_.size(); i++ ){
        visited_[i] = false;
    }
}
void Ant::addToVisited(unsigned int cellID) {
    visited_[cellID] = true;
}

void Ant::setSearchTrue(){
    search = true;
}
void Ant::setSearchFalse(){
    search = false;
}

bool Ant::isSearching() {
    return search; // goal is reached only if search is false
}





