//
// Created by fabio on 09/09/2021.
//

// AS algorithm

#ifndef ANT_COLONY_PATH_PLANNER_ACO_H
#define ANT_COLONY_PATH_PLANNER_ACO_H
// ACO dependencies
#include "Path.h"
#include "Ant.h"
#include "OccupancyGrid.h"
// ROS dependencies
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

// C++ dependencies
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

class ACO {
public:
    /***** CONSTRUCTOR *****/
    ACO();
    ACO(double alpha, double beta, double evaporationRate, double initialPheromoneValue, int antNumber, int iterationsLimit);
    /***** DESTRUCTOR *****/
    ~ACO();

    /***** MUTATORS *****/
    // Setter
    void setPheromoneMatrix(vector<vector<double>> pheromoneMatrix);
    void setTotalMatrix(vector<vector<double>> total);
    void setAlpha(double alpha);
    void setBeta(double beta);
    void setEvaporationRate(double evaporationRate);
    void setInitialPheromoneValue(double initialPheromoneValue);
    void setAntNumber(int antNumber);
    void setIterationsLimit(int it);

    // Getter
    vector<vector<double>> getPheromoneMatrix();
    vector<vector<double>> getTotal();
    double getAlpha();
    double getBeta();
    double getEvaporationRate();
    double getInitialPheromoneValue();
    int getAntNumber();
    int getIterationsLimit();

    /***** INITIALIZATION *****/
    void initializePheromoneMatrix(OccupancyGridMap* map);
    void initializeTotalMatrix(OccupancyGridMap* map);

    /***** UTILITIES *****/
    /**
     * @brief Get a vector containing the ID of the cells around the current one.
     * @param map
     * @param currCell
     * @return Vector of neighbours (not necessarily free).
     */
    vector<unsigned int> getNeighbours(OccupancyGridMap* map, unsigned int cellID);
    /**
     * @brief Choose the best neighbour of current ant position
     * @param map
     * @param ant
     * @return Index of the choosen cell
     */
    unsigned int chooseNeighbour(OccupancyGridMap* map, Ant* ant);
    /**
     * @brief Tells if a cell is part of the given path
     * @param path
     * @param cellID
     * @return True if visited.
     */
    bool isVisited(Ant* ant, unsigned int cellID);
    /**
     * @brief Update statistics infos such as best ant, best ant of current iteration, iteration where the best path was found.
     * @param iteration
     */
    void updateStatistics(OccupancyGridMap* map, int iteration);
    /**
     * @brief Search the ant that made the less expensive path
     * @return Index of the ant in the colony vector.
     */
    int findBestAnt();
    /**
     * @brief Generate a random value
     * @param minBound
     * @param maxBound
     * @return
     */
    double getRandom(double minBound, double maxBound);
    /**
     * @brief check if all the ants reached the goal point
     * @param map
     * @param goalCellID
     * @return
     */
    bool endSearchCondition(unsigned int goalCellID, int searchLimit);
    /**
     * @brief chooses for an ant as the next cell the one with maximal value of heuristic information times pheromone
     * @param ant
     * @return cell index
     */
    unsigned int chooseBestNext(OccupancyGridMap* map, Ant* ant);

    void placeAnts(OccupancyGridMap* map, unsigned int startCellID);

    /***** PHEROMONE + HEURISTIC *****/
    /**
     * @brief Update the pheromoneMatrix_
     * @param map
     */
    void pheromoneTrailUpdate(OccupancyGridMap* map);
    /**
     * @brief Ant System's algorithm pheromone trail update
     * @param map
     */
    void asUpdate(OccupancyGridMap* map);
    /**
     * @brief Called by asUpdate for each ant. Deposit the pheromone on the constructed paths.
     * @param map
     * @param ant
     */
    void globalPheromoneUpdate(OccupancyGridMap* map, Ant* ant);
    /**
     * @ Put together heuristic and pheromone informations and update total_ matrix.
     * @param map
     */
    void computeTotalInformation(OccupancyGridMap* map);

    /***** PATH PLANNING *****/
    /**
     * @brief Construct the path of the k-esim ant.
     * @param map
     * @param ant
     * @param startCellID
     * @param goalCellID
     */
    void constructSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID);
    void constructParallelSolutions(OccupancyGridMap* map, unsigned int goalCellID);
    /**
     * @brief Compute paths for every ant
     * @param map
     * @param startCellID
     * @param goalCellID
     * @return The best path found after iterationsLimit_ iterations
     */
    Path* computePaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID);
    Path* computeParallelPaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID);

    /***** LOCAL SEARCH *****/
    // DA DISCUTERE
    void localSearch();
    void twoOptFirst(Path* path);

private:
    vector<vector<double>> pheromoneMatrix_; // Pheromone matrix.
    vector<vector<double>> total_; // Pheromone + Heuristic matrix
    vector<Ant*> colony; // Ant colony

    Ant* bestAntOfIteration; // Defined as the ant that built the shortest path in a single iteration
    Ant* bestAnt; // The ant that built the shortest path in all iterations
    unsigned int bestAntIndex;

    double alpha_; // Pheromone information weight.
    double beta_; // Heuristic information weight.
    double evaporationRate_; // Pheromone evaporation rate.
    double initialPheromoneValue_; // Initialization value.
    int antNumber_; // Size of the Colony
    int iterationsLimit_; // Number of iterations wanted
    int bestIteration; // Iteration in which the best path has been found

    ofstream traceFile;
};


#endif //ANT_COLONY_PATH_PLANNER_ACO_H
