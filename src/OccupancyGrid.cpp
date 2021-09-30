/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration.

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact:
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "OccupancyGrid.h"
#include <limits.h>
#include <ros/ros.h>
#include <string.h>
#include <stdlib.h>
#include <sstream> // stringstream

using namespace std;

//constructors
OccupancyGridMap::OccupancyGridMap() : width(0), height(0), resolution(0){
}

//destructors
OccupancyGridMap::~OccupancyGridMap() = default;

OccupancyGridMap::OccupancyGridMap(unsigned int w, unsigned int h, double res, unsigned int **mapMatrix, int obsSize,
                                   float obsRatio){//: Map(w, h, res, mapMatrix, obsSize, obsRatio)
    setWidth(w);
    setHeight(h);
    setResolution(res);
    setMapLayout(mapMatrix);
    setObstacleSize(obsSize);
    setObstacleRatio(obsRatio);
    mapLayout = new unsigned int*[h];
    for( int i = 0; i < (int)height; i++ ){
        mapLayout[i] = new unsigned int[(int)width];
    }
}

OccupancyGridMap::OccupancyGridMap(OccupancyGridMap *map){ //: Map(map)
    setWidth(map->getWidth());
    setHeight(map->getHeight());
    setResolution(map->getResolution());
    setMapLayout(map->getMapLayout());
    setObstacleSize(map->getObstacleSize());
    setObstacleRatio(map->getObstacleRatio());
}

/***** MUTATORS *****/
// Setter
void OccupancyGridMap::setWidth(unsigned int w) {
    width = ((w < 0) ? 0 : w);
}
void OccupancyGridMap::setHeight(unsigned int h) {
    height = ((h < 0) ? 0 : h);
}
void OccupancyGridMap::setResolution(double res) {
    resolution = ((res < 0) ? 0 : res);
}
void OccupancyGridMap::setMapLayout(unsigned int **mapMatrix) {
    mapLayout = mapMatrix;
}
void OccupancyGridMap::setObstacleSize(int obsSize) {
    obstacleSize = obsSize;
}
void OccupancyGridMap::setObstacleRatio(float obsRatio) {
    obstacleRatio = obsRatio;
}
// Getter
unsigned int OccupancyGridMap::getWidth() const {
    return width;
}
unsigned int OccupancyGridMap::getHeight() const {
    return height;
}
double OccupancyGridMap::getResolution() const {
    return resolution;
}
unsigned int **OccupancyGridMap::getMapLayout() {
    return mapLayout;
}
int OccupancyGridMap::getObstacleSize() const {
    return obstacleSize;
}
float OccupancyGridMap::getObstacleRatio() const {
    return obstacleRatio;
}

/***** MAP LAYOUT *****/
// Update
void OccupancyGridMap::setObstacle(unsigned int i, unsigned int j) {
    getMapLayout()[i][j] = OBSTACLE;
}
void OccupancyGridMap::setFree(unsigned int i, unsigned int j) {
    getMapLayout()[i][j] = FREE;
}
void OccupancyGridMap::setUnknown(unsigned int i, unsigned int j) {
    getMapLayout()[i][j] = UNKNOWN;
}
void OccupancyGridMap::setValid(unsigned int i, unsigned int j) {
    getMapLayout()[i][j] = VALID;
}
void OccupancyGridMap::setCell(unsigned int i, unsigned int j, unsigned int occupancy) {
    getMapLayout()[i][j] = occupancy;
}
// Check
// With coordinates
bool OccupancyGridMap::isObstacle(unsigned int i, unsigned int j) {
    return (getMapLayout()[i][j] == OBSTACLE);
}
bool OccupancyGridMap::isFree(unsigned int i, unsigned int j) {
    // cout << "Map Layout value: " << getMapLayout()[i][j] << endl;
    return (getMapLayout()[i][j] == FREE);
}
bool OccupancyGridMap::isUnknown(unsigned int i, unsigned int j) {
    return (getMapLayout()[i][j] == UNKNOWN);
}
bool OccupancyGridMap::isValid(unsigned int i, unsigned int j) {
    return (getMapLayout()[i][j] <= VALID);
}
// With index
bool OccupancyGridMap::isObstacle(unsigned int CellID) {
    unsigned int mx = 0, my = 0;
    indexToCells(CellID, mx, my);
    return (getMapLayout()[mx][my] == OBSTACLE);
}
bool OccupancyGridMap::isFree(unsigned int CellID) {
    unsigned int mx = 0, my = 0;
    indexToCells(CellID, mx, my);
    return (getMapLayout()[mx][my] == FREE);
}
bool OccupancyGridMap::isUnknown(unsigned int CellID) {
    unsigned int mx = 0, my = 0;
    indexToCells(CellID, mx, my);
    return (getMapLayout()[mx][my] == UNKNOWN);
}
bool OccupancyGridMap::isValid(unsigned int CellID) {
    unsigned int mx = 0, my = 0;
    indexToCells(CellID, mx, my);
    return (getMapLayout()[mx][my] <= VALID);
}
unsigned int OccupancyGridMap::getCellCost(unsigned int mx, unsigned int my){
    return getMapLayout()[mx][my];
}
unsigned int OccupancyGridMap::getCellCost(unsigned int CellID){
    unsigned int mx = 0, my = 0;
    indexToCells(CellID, mx, my);
    return getCellCost(mx, my);
}
/***** MOVE METHODS *****/
float OccupancyGridMap::getMoveCost(unsigned int i1, unsigned int j1, unsigned int i2, unsigned int j2) {
    float moveCost = INFINIT_COST; //start cost with maximum value. Change it to real cost of cells are connected
    //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) ||
        (j2 == j1 - 1 && i2 == i1 + 1)) {
        moveCost = DIAGONAL_MOVE_COST;
    }
        //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
    else {
        if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) ||
            (i1 == i2 && j2 == j1 + 1)) {
            moveCost = MOVE_COST;
        }
    }
    return moveCost;
}
float OccupancyGridMap::getMoveCost(unsigned int CellID1, unsigned int CellID2) {
    unsigned int mx1 = 0, mx2 = 0, my1 = 0, my2 = 0;
    indexToCells(CellID1, mx1, my1);
    indexToCells(CellID1, mx2, my2);

    return getMoveCost(mx1, mx1, my2, my2);
}

/***** UTILITIES *****/
unsigned int OccupancyGridMap::getCellIndex(unsigned int mx, unsigned int my) const{
    return (my * getWidth()) + mx;
}
void OccupancyGridMap::indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const{
    my = index / getWidth();
    mx = index - (my * getWidth());
}

/***** IMPORT/EXPORT LAYOUT *****/
void OccupancyGridMap::importMapLayout(string path_name, const char *file_name) {
    int row = 0, col = 0, numrows = 0, numcols = 0, pixel_maxi = 0;
    string path_loc = path_name + file_name; //the location of the file
    ifstream infile(path_loc.c_str());
    stringstream ss;
    string inputLine = "";
    if (!infile) {
        //file couldn't be opened
        cerr << "[I/O Error] The map file could not be opened. \nCheck file name and/or path" << endl;
        exit(1);
    } else {
        // First line : version
        getline(infile, inputLine);
        if (inputLine.compare("P2") != 0)
            cerr << "Version error" << endl;
        else
            cout << "Version : " << inputLine << endl;

        // Second line : comment
        getline(infile, inputLine);
        cout << "Comment : " << inputLine << endl;
        int indexObstacleRatio = inputLine.find("Obstacle Ratio=", 0);
        int index1 = inputLine.find(" ", indexObstacleRatio + 10);
        string obstacleRatio = inputLine.substr(indexObstacleRatio + 15, index1 - (indexObstacleRatio + 15));
        setObstacleRatio(atof(obstacleRatio.c_str()));

        int indexObstacleSize = inputLine.find("Obstacle Size=", 0);
        int index2 = inputLine.find(" ", indexObstacleSize + 10);
        string obstacleSize = inputLine.substr(indexObstacleSize + 14, index2 - (indexObstacleSize + 14));
        setObstacleSize(atoi(obstacleSize.c_str()));

        // Continue with a stringstream
        ss << infile.rdbuf();
        // Third line : size
        ss >> numcols >> numrows;
        cout << numcols << " columns and " << numrows << " rows" << endl;

        //the forth line
        ss >> pixel_maxi;
        cout << pixel_maxi << endl;

        setHeight(numrows);
        setWidth(numcols);
        mapLayout = (unsigned int **) malloc(sizeof(unsigned int *) * getHeight());
        for (int i = 0; i < getWidth(); i++)
            mapLayout[i] = (unsigned int *) malloc(sizeof(unsigned int) * getWidth());

        // Following lines : data
        for (row = 0; row < numrows; ++row)
            for (col = 0; col < numcols; ++col)
                ss >> getMapLayout()[row][col];

        // Now print the MapLayout to see the result
        for (row = 0; row < numrows; ++row) {
            for (col = 0; col < numcols; ++col) {
                // cout << getMapLayout()[row][col] << " ";
            }
            //cout << endl;
        }

        infile.close();
    }
}
void OccupancyGridMap::exportMapLayout(const char *file_name, int **mapMatrix) {
    ofstream file(file_name, ios::trunc);
    /*find the number of obstacle cells*/
    double sumObs = 0;
    for (int i = 0; i < getHeight(); i++) {
        for (int j = 0; j < getWidth(); j++) {
            if (getMapLayout()[i][j] == OBSTACLE)
                sumObs++;
        }
    }
    /*write the PGM file*/
    file << "P2";
    file << endl;
    file << "#Generated by exportMapLayout in OccupancyGridMap.h. Obstacle Ratio=";
    file << sumObs / (getWidth() * getHeight());
    file << endl;
    file << getWidth();
    file << " ";
    file << getHeight();
    file << endl;
    file << "100";
    file << endl;
    for (int i = 0; i < getHeight(); i++) {
        for (int j = 0; j < getWidth(); j++) {
            file << mapMatrix[i][j];
            if (j < getWidth() - 1)
                file << " ";
        }
        if (i < getHeight() - 1)
            file << endl;
    }
}
