//
// Created by fabio on 27/12/21.
//

#ifndef ANT_COLONY_PATH_PLANNER_ACO_V2_H
#define ANT_COLONY_PATH_PLANNER_ACO_V2_H
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

class ACO_v2 {
public:
    /***** CONSTRUCTOR *****/
    ACO_v2();
    ACO_v2(double alpha, double beta, double evaporationRate, int antNumber,
           int iterationsLimit, long int seed, vector<Ant*> &colony, vector<vector<double>>& pheromoneMatrix, vector<vector<double>>& total);
    /***** DESTRUCTOR *****/
    ~ACO_v2();

    /***** MUTATORS *****/
    // Setter
    void setPheromoneMatrix(vector<vector<double>>& pheromoneMatrix);
    void setTotalMatrix(vector<vector<double>>& total);
    void setAlpha(double alpha);
    void setBeta(double beta);
    void setEvaporationRate(double evaporationRate);
    void setAntNumber(int antNumber);
    void setIterationsLimit(int it);

    // Getter
    vector<vector<double>> getPheromoneMatrix();
    vector<vector<double>> getTotal();
    double getAlpha();
    double getBeta();
    double getEvaporationRate();
    int getAntNumber();
    int getIterationsLimit();

    /***** UTILITIES *****/
    /**
     * @brief Get a vector containing the ID of the cells around the current one.
     * @param map
     * @param currCell
     * @return Vector of neighbours (not necessarily free).
     */
    vector<unsigned int> getNeighbours(OccupancyGridMap* map, Ant* ant);
    /**
     * @brief Choose the best neighbour of current ant position
     * @param map
     * @param ant
     * @return Index of the choosen cell
     */
    unsigned int chooseNeighbour(OccupancyGridMap* map, vector<unsigned int> neighbours);
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
     * @brief Copy ant a1 in ant a2
     * @param a1
     * @param a2
     */
    void copyFromTo ( Ant* a1, Ant* a2);
    /**
     * @brief Generate a random value
     * @param minBound
     * @param maxBound
     * @return
     */
    double getRandom(double minBound, double maxBound);
    /**
     * @brief Generate a random value between 0 and 1
     * @param minBound
     * @param maxBound
     * @return
     */
    double getRandom01(long* idum);
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

    double euclideanDistance(unsigned int x1, unsigned int y1,unsigned int x2, unsigned int y2);

    void placeAnts(OccupancyGridMap* map, unsigned int startCellID);

    /***** PHEROMONE + HEURISTIC *****/
    /**
     * @brief Update the pheromoneMatrix_
     * @param map
     */
    void pheromoneTrailUpdate(OccupancyGridMap* map, unsigned int goalCellID);
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
    void computeTotalInformation(OccupancyGridMap* map, unsigned int goalCellID);

    /***** PATH PLANNING *****/
    /**
     * @brief Construct the path of the k-esim ant.
     * @param map
     * @param ant
     * @param startCellID
     * @param goalCellID
     */
    void constructSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID);
    void constructParallelSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID);
    /**
     * @brief Compute paths for every ant
     * @param map
     * @param startCellID
     * @param goalCellID
     * @return The best path found after iterationsLimit_ iterations
     */
    vector<unsigned int> computePaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID);
    vector<unsigned int> computeParallelPaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID);

    /***** LOCAL SEARCH *****/
    // DA DISCUTERE
    void localSearch();
    void twoOptFirst(Path* path);

private:
    vector<vector<double>> pheromoneMatrix_; // Pheromone matrix.
    vector<vector<double>> total_; // Pheromone + Heuristic matrix
    vector<Ant*> colony_; // Ant colony

    Ant* bestAntOfIteration; // Defined as the ant that built the shortest path in a single iteration
    Ant bestAnt; // The ant that built the shortest path in all iterations
    Ant restartBestAnt; // Restart best ant
    unsigned int bestAntIndex;

    double alpha_; // Pheromone information weight.
    double beta_; // Heuristic information weight.
    double evaporationRate_; // Pheromone evaporation rate.
    int antNumber_; // Size of the Colony
    int iterationsLimit_; // Number of iterations wanted
    int bestIteration; // Iteration in which the best path has been found
    int restartBestIteration;

    long int seed_; // For random number generator

    ofstream traceFile;
};


#endif //ANT_COLONY_PATH_PLANNER_ACO_V2_H
