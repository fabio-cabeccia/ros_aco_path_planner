//
// Created by fabio on 14/09/2021.
//

#ifndef ANT_COLONY_PATH_PLANNER_OCCUPANCYGRID_H
#define ANT_COLONY_PATH_PLANNER_OCCUPANCYGRID_H
// C++ dependencies
#include <limits.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

using namespace std;

/**
* @class OccupancyGridMap
* @brief A class that represents an occupancy grid map
*/
class OccupancyGridMap //: public Map
{

public:
    //define constants
    static const int FREE = 0;       //!< for free cell
    static const int OBSTACLE = 100; //!< for obstacles
    static const int UNKNOWN = -1;   //!< for unknown cell
    static const int VALID = 16; //!< free threshold

    static constexpr float MOVE_COST = 1.0;          //!< horizental or vertical move
    static constexpr float DIAGONAL_MOVE_COST = 1.4; //!< Diagonal move

    static constexpr float INFINIT_COST = INT_MAX; //!< cost of non connected nodes

    /**
     * @brief default constructor to initialize map
     */
    OccupancyGridMap();

    /**
     * @brief default destructor
     */
    ~OccupancyGridMap();

    /**
    * @brief four-argument constructor to initialize map
    */
    OccupancyGridMap(unsigned int w, unsigned int h, double res, unsigned int **mapMatrix, int obsSize, float obsRatio);

    /**
    * @brief constructor that loads a map from a file
    */
    OccupancyGridMap(ifstream *mpgMapFile);

    /**
    * @brief Copy constructor for the OccupancyGridMapGridMap
    */
    OccupancyGridMap(OccupancyGridMap *map);

    /***** MUTATORS *****/
    // Setters
    void setWidth(unsigned int w);
    void setHeight(unsigned int h);
    void setResolution(double res);
    void setMapLayout(unsigned int **mapMatrix);
    void setObstacleSize(int obsSize);
    void setObstacleRatio(float obsRatio);
    // Getters
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    double getResolution() const;
    unsigned int **getMapLayout();
    int getObstacleSize() const;
    float getObstacleRatio() const;

    /***** MAP LAYOUT *****/
    // Update
    void setObstacle(unsigned int i, unsigned int j);
    void setFree(unsigned int i, unsigned int j);
    void setUnknown(unsigned int i, unsigned int j);
    void setValid(unsigned int i, unsigned int j);
    void setCell( unsigned int i, unsigned int j, unsigned int occupancy);
    // Check
    /**
    * @brief it is used to check if the cell is an obstacle cell
    * @param i
    * @param j
    * @return true if the cell is an Obstacle
    */
    bool isObstacle(unsigned int i, unsigned int j);
    /**
    * @brief it is used to check if the cell is a free cell
    * @param i
    * @param j
    * @return true if the cell is Free
    */
    bool isFree(unsigned int i, unsigned int j);
    /**
    * @brief it is used to check if the cell is unknown cell
    * @param i
    * @param j
    * @return true if the cell is Unknown
    */
    bool isUnknown(unsigned int i, unsigned int j);
    /**
    * @brief it is used to check if the cell is valid cell
    * @param i
    * @param j
    * @return true if the cell is VALID
    */
    bool isValid(unsigned int i, unsigned int j);
    /**
    * @brief it is used to check if the cell is a free cell
    * @param CellID
    * @return true if the cell is Free
    */
    bool isFree(unsigned int CellID);
    /**
    * @brief it is used to check if the cell is an obstacle cell
    * @param CellID
    * @return true if the cell is an Obstacle
    */
    bool isObstacle(unsigned int CellID);
    /**
    * @brief it is used to check if the cell is unknown cell
    * @param CellID
    * @return true if the cell is Unknown
    */
    bool isUnknown(unsigned int CellID);
    /**
    * @brief it is used to check if the cell is unknown cell
    * @param CellID
    * @return true if the cell is Unknown
    */
    bool isValid(unsigned int CellID);
    unsigned int getCellCost(unsigned int mx, unsigned int my);
    unsigned int getCellCost(unsigned int CellID);
    /***** MOVE METHODS *****/
    /**
    * @brief it is used to get the move cost between 2 cells
    * @param i1
    * @param j1
    * @param i2
    * @param j2
    * @return move cost
    */
    float getMoveCost(unsigned int i1, unsigned int j1, unsigned int i2, unsigned int j2);
    /**
    * @brief it is used to get the move cost between 2 cells
    * @param CellID1
    * @param CellID2
    * @return move cost
    */
    float getMoveCost(unsigned int CellID1, unsigned int CellID2);

    /***** UTILITIES *****/
    /**
      * @brief it is used to get the index of the cell to be used in Path.
      * @param i
      * @param j
      * @return cell index
      */
    unsigned int getCellIndex(unsigned int i, unsigned int j) const;
    /**
     * @brief Translate the index in coordinates
     * @param index
     * @param mx
     * @param my
     */
    void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const;

    /***** IMPORT/EXPORT LAYOUT *****/
    /**
      * @brief it is used to read the MapLayout from a file
      * @param path_name
      * @param file_name
      */
    void importMapLayout(string path_name, const char *file_name);
    /**
      * @brief it is used to write the MapLayout in a file
      * @param file_name
      * @param mapMatrix
      */
    void exportMapLayout(const char *file_name, int **mapMatrix);

private:
    unsigned int width;           //!<  Map width [cells]
    unsigned int height;          //!<  Map height [cells]
    double resolution;    //!< The map resolution [m/cell]
    unsigned int **mapLayout;     //!<  the map data structure that contains all cells of the environment
    int obstacleSize;    //!< The size of the obstacles
    float obstacleRatio; //!< The ratio of obstacles in the map
};

#endif //ANT_COLONY_PATH_PLANNER_OCCUPANCYGRID_H
