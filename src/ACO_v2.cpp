//
// Created by fabio on 27/12/21.
//

#include "ACO_v2.h"
#include "Ant.h"
#include "OccupancyGrid.h"
#include "Path.h"

#include <ros/ros.h>

#include <cmath>
#include <random>
#include <vector>

/* constants for a random number generator, for details see numerical recipes in C */
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836

using namespace std;
/***** CONSTRUCTORS/DESTRUCTORS *****/
ACO_v2::ACO_v2() = default;
ACO_v2::ACO_v2(double alpha, double beta, double evaporationRate, int antNumber,int iterationsLimit, long int seed, vector<Ant*> &colony,
               vector<vector<double>>& pheromoneMatrix, vector<vector<double>>& total) {
    setAlpha(alpha);
    setBeta(beta);
    setEvaporationRate(evaporationRate);
    setAntNumber(antNumber);
    setIterationsLimit(iterationsLimit);
    bestIteration = 0;
    colony_ = colony;
    pheromoneMatrix_ = pheromoneMatrix;
    total_ = total;
    seed_ = seed;
}
ACO_v2::~ACO_v2(void) = default;

/***** MUTATORS *****/
void ACO_v2::setPheromoneMatrix(vector<vector<double>>& pheromoneMatrix) {
    pheromoneMatrix_ = pheromoneMatrix;
}
void ACO_v2::setTotalMatrix(vector<vector<double>>& total) {
    total_ = total;
}
void ACO_v2::setAlpha(double alpha) {
    alpha_ = alpha;
}
void ACO_v2::setBeta(double beta) {
    beta_ = beta;
}
void ACO_v2::setEvaporationRate(double evaporationRate) {
    evaporationRate_ = evaporationRate;
}
void ACO_v2::setAntNumber(int antNumber) {
    antNumber_ = antNumber;
}
void ACO_v2::setIterationsLimit(int it) {
    iterationsLimit_ = it;
}

vector<vector<double>> ACO_v2::getPheromoneMatrix() {
    return pheromoneMatrix_;
}
vector<vector<double>> ACO_v2::getTotal(){
    return total_;
}
double ACO_v2::getAlpha() {
    return alpha_;
}
double ACO_v2::getBeta() {
    return beta_;
}
double ACO_v2::getEvaporationRate() {
    return evaporationRate_;
}
int ACO_v2::getAntNumber() {
    return antNumber_;
}
int ACO_v2::getIterationsLimit() {
    return iterationsLimit_;
}

/***** UTILITIES *****/
vector<unsigned int> ACO_v2::getNeighbours(OccupancyGridMap* map, Ant* ant) {
    unsigned int mx = 0;
    unsigned int my = 0;
    unsigned int debugIndex = 0;
    map->indexToCells(ant->getConstructedPath()->getPath().back(), mx, my);
    vector<unsigned int> neighbourCells;

    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++) {
            // With i = 0 and j = 0 current cell is obtained. In that case do nothing
            if( i == 0 && j == 0 )
                continue;
            // Check if the coordinates are legal
            if ( (mx + i > 0) &&  (mx + i < map->getHeight()) && (my + j > 0) && (my + j < map->getWidth()))
                if ( map->isValid(map->getCellIndex(mx + i, my + j))  && !ant->isVisited(map->getCellIndex(mx + i, my + j)) )
                    neighbourCells.emplace_back(map->getCellIndex(mx + i, my + j));
        }
    return neighbourCells;
}
unsigned int ACO_v2::chooseNeighbour(OccupancyGridMap* map, vector<unsigned int> neighbours) {
    // Define a probability vector. Its size depends on the amount of neighbours
    vector<double> probabilities;
    //probabilities.reserve(neighbours.size());
    vector<double> cumulativeProbabilities;
    //cumulativeProbabilities.reserve(probabilities.size());
    // Store the cumulative probability
    double probSum = 0.0;
    // Used to check the selection condition
    double partialSum = 0;
    // Output
    unsigned int chosen = 0;
    // Temporary values to navigate the map
    unsigned int mx = 0;
    unsigned int my = 0;
    int l = 0;

    for (auto i: neighbours) { // For each neighbour cell check if free
        map->indexToCells(i, mx, my);
        probabilities.emplace_back(total_[mx][my]); // Total computed cost of the cell
        probSum += total_[mx][my];
        //ROS_INFO("Ant %d, TIMES %d, VALID %d, VISITED %d", ant->getName(), l, map->isValid(i), isVisited(ant, i));
    }
    // Normalize the probabilities
    if ( probSum != 0 ){
        for(double & p : probabilities){
            p = p / probSum;
        }
    }

    // Prepare cumulative probabilities vector
    cumulativeProbabilities.reserve(probabilities.size());
for( int i = 0; i < probabilities.size(); i++ ){
        cumulativeProbabilities.emplace_back( probabilities[i-1] + probabilities[i] );
    }

    // Generate a random number
    double rand = getRandom01(&seed_);

    if( rand >= 0 && rand <= cumulativeProbabilities[0] ){
        chosen = neighbours[0];
    }
    else if ( rand >= cumulativeProbabilities.back() && rand <= 1){
        chosen = neighbours.back();
    }
    else{
        for (int i = 0; i < cumulativeProbabilities.size(); i++){
            if (rand >= cumulativeProbabilities[i] && rand <= cumulativeProbabilities[i + 1]){
                long double difference1 = rand - cumulativeProbabilities[i];
                long double difference2 = cumulativeProbabilities[i + 1] - rand;
                if (difference1 < difference2)
                    chosen = neighbours[i];
                else
                    chosen = neighbours[i + 1];
            }
        }
    }

    /* // Choose the neighbour
    if( probSum <= 0 ){ // No feasible neighbour cells.
        //ROS_INFO("STUCK");
        // Delete the last element of the path since it leads to a deadlock
        ant->getConstructedPath()->popBack();
        // Update the cost
        ant->getConstructedPath()->computeCost(map);
        // Update the ant position
        ant->setPosition(ant->getConstructedPath()->getPath().back());
        // Recursive call with updated ant position, path cells and path cost.
        chosen = chooseBestNext(map, ant);
        chosen = chooseBestNext(map, ant);
    }
    else{ // At least one of the neighbours is eligible. Select one according to the probabilities rule.
        // Select a random01 value
        double random = getRandom(0, probSum);//getRandom01(&seed_);
        //random *= probSum;
        // Index for updating partialSum
        int i = 0;
        // Set the initial value for partial sum
        partialSum = probabilities[0].second;
        while ( partialSum <= random ){
            //ROS_INFO("I: %d Partial sum %.1f, ProbSum %.1f, RANDOM %.1f", i, partialSum, probSum, random);
            i++;
            partialSum += probabilities[i].second;
            //ROS_INFO("I: %d Partial sum %.1f, ProbSum %.1f, RANDOM %.1f", i, partialSum, probSum, random);
        }

        if ( i == neighbours.size()) {
            chosen = chooseBestNext(map, ant);
        }
        else {

            chosen = probabilities[i].first;
            map->indexToCells(chosen, mx, my);
            //ROS_INFO("CHOSEN NORMALMENTE %d, %d", mx, my);
        }
    } */

    return chosen;
}
void ACO_v2::updateStatistics(OccupancyGridMap* map, int iteration) {
    // Find best ant of the current iteration based on its path length.
    unsigned int iterationBestAnt = findBestAnt();

    // Update bestAnt if a best ant of iteration has been found
    if ( iterationBestAnt != -1 ){
        //ROS_INFO("Best ant of iteration: %d, path length: %zu, cost: %f", colony_[iterationBestAnt]->getName(),
                 colony_[iterationBestAnt]->getConstructedPath()->getPath().size(),
                 colony_[iterationBestAnt]->getConstructedPath()->getCost();
        if ( bestAnt.getConstructedPath()->getCost() == 0 || colony_[iterationBestAnt]->getConstructedPath()->getCost() <
        bestAnt.getConstructedPath()->getCost() ) {
            copyFromTo(colony_[iterationBestAnt], &bestAnt);
            copyFromTo(colony_[iterationBestAnt], &restartBestAnt);
            bestIteration = iteration;
            restartBestIteration = iteration;
        }
        if (colony_[iterationBestAnt]->getConstructedPath()->getCost() <
            restartBestAnt.getConstructedPath()->getCost()) {
            ACO_v2::copyFromTo(colony_[iterationBestAnt], &restartBestAnt);
            restartBestIteration = iteration;
            ROS_INFO("Restart best: %f, restartBestIteration %d\n", restartBestAnt.getConstructedPath()->getCost(),
                     restartBestIteration);
        }


        //ROS_INFO("Best ant: %d, path length: %zu, cost: %.1f", bestAnt.getName(),
                 bestAnt.getConstructedPath()->getPath().size(),
                 bestAnt.getConstructedPath()->getCost();
    }
    else{
        //ROS_INFO("All ants have been stuck during iteration %d.", iteration);
    }
}
int ACO_v2::findBestAnt() {
    unsigned long min;
    int best;

    min = -1;
    best = -1;

    for(int k = 0; k < colony_.size(); k++){
        if ( !colony_[k]->isSearching() && (min == -1 || colony_[k]->getConstructedPath()->getPath().size() < min) ) {
            min = colony_[k]->getConstructedPath()->getPath().size();
            best = k;
        }
    }
    return best;
}
void ACO_v2::copyFromTo ( Ant* a1, Ant* a2 ){
    a2->getConstructedPath()->clearPath();
    a2->clearVisited();
    a2->getConstructedPath()->setCost(a1->getConstructedPath()->getCost());
    //ROS_INFO("cost best ant %f", a1->getConstructedPath()->getCost());
    a2->getConstructedPath()->setPath(a1->getConstructedPath()->getPath());
    //ROS_INFO("dim path best ant of iteration %zu", a1->getConstructedPath()->getPath().size());

    vector<bool> temp;
    for ( auto i: a1->getVisited() ) {
        temp.emplace_back(i);
    }


    a2->setVisited(temp);
    a2->setName(a1->getName());
    //ROS_INFO("dim path best ant %zu", a2->getConstructedPath()->getPath().size());
    //ROS_INFO("test3");
}
double ACO_v2::getRandom(double minBound, double maxBound){
    random_device rd;
    default_random_engine rdGenerator( rd() );
    uniform_real_distribution<> rdValue(minBound, maxBound);

    return rdValue(rdGenerator);
}
double ACO_v2::getRandom01(long* idum){
    long k;
    double ans;

    k =(*idum)/IQ;
    *idum = IA * (*idum - k * IQ) - IR * k;
    if (*idum < 0 ) *idum += IM;
    ans = AM * (*idum);
    return ans;
}
bool ACO_v2::endSearchCondition(unsigned int goalCellID, int searchCondition){
    while( searchCondition <= 10000 ) {
        for (auto k: colony_) {
            if (k->getPosition() != goalCellID)
                return false;
        }
    }
    return true;
}
unsigned int ACO_v2::chooseBestNext(OccupancyGridMap* map, Ant* ant){
    double value_best = -1.;
    unsigned int nxtCell = -1;
    unsigned int mx = 0, my = 0;
    int cnt = 0;
   /* vector<unsigned int> neighbours = getNeighbours(map, neighbours);

    for( auto i: neighbours ){
        if( map->isFree(i) && !ant->isVisited(i)){
            // ROS_INFO("NEIGHBOURS %d in choseBestNext is Free", i);
            map->indexToCells(i, mx, my);
            if( value_best < total_[mx][my] ){ // Total values are always >= 0
                value_best = total_[mx][my];
                nxtCell = i;
            }
        }
        //else
        //cnt++;
    }
    if( nxtCell == -1 ){

    }
    if( cnt == neighbours.size() ){
        return -1;
    } */
    return nxtCell;
}
double ACO_v2::euclideanDistance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2){
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}
void ACO_v2::placeAnts(OccupancyGridMap* map, unsigned int startCellID){
    //ROS_INFO("Placing ants");

    for( auto k: colony_ ){
        // Clear the path
        //ROS_INFO("Size colony %zu", colony_.size());
        //ROS_INFO("VISITED PRIMA DI CLEAR %zu", k.getVisited().size());
        k->getConstructedPath()->clearPath();
        k->clearVisited();
        k->getConstructedPath()->setCost(0);
        k->setSearchFalse();

        //ROS_INFO("CLEARED PATH LENGTH %zu, VISITED CLEARED %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
        // Place the ant at start cell
        k->getConstructedPath()->insertCell(map, startCellID);
        k->addToVisited(startCellID);
        k->setPosition(startCellID);
        k->setSearchTrue();
        //ROS_INFO("Formica %d piazzata in pos %d", k.getName(), k.getPosition());
        // ROS_INFO("START CELL AGGIUNTA %d", startCellID);
        //ROS_INFO("PATH AFTER FIRST CELL %zu cell inserted %d", k.getConstructedPath()->getPath().size(),  k.getConstructedPath()->getPath()[0]);
    }
}

/***** PHEROMONE + HEURISTIC *****/
void ACO_v2::pheromoneTrailUpdate(OccupancyGridMap* map, unsigned int goalCellID) {
    // Coordinates of the cell
    unsigned int mx = 0;
    unsigned int my = 0;
    // Pheromone evaporation
    for( int i = 0; i < map->getHeight(); i++ ){
        for( int j = 0; j < map->getWidth(); j++ ){
            pheromoneMatrix_[i][j] *= (1 - evaporationRate_);
        }
    }

    // Ant System pheromone trail deposit
    asUpdate(map);

    // Compute combined information of pheromone trail and heuristic
    computeTotalInformation(map, goalCellID);
}
void ACO_v2::asUpdate(OccupancyGridMap* map) {
    for( auto k: colony_ ){
        globalPheromoneUpdate(map, k);
    }
}
void ACO_v2::globalPheromoneUpdate(OccupancyGridMap* map, Ant* ant) {
    // Cells coordinates
    unsigned int mx = 0;
    unsigned int my = 0;
    // Reinforce only the edges used in ant's solution
    double d_tau = 1.0 / ant->getConstructedPath()->getCost();

    //ROS_INFO("test");
    for( unsigned int i: ant->getConstructedPath()->getPath() ) {
        // Update mx and my
        map->indexToCells(i, mx, my);
        pheromoneMatrix_[mx][my] += d_tau;
    }
}
void ACO_v2::computeTotalInformation(OccupancyGridMap* map, unsigned int goalCellID) {
    unsigned int mx = 0, my = 0;
    map->indexToCells(goalCellID, mx, my);

    double heuristic = 0;
    for (unsigned int i = 0; i < map->getHeight(); i++) {
        for (unsigned int j = 0; j < map->getWidth(); j++) {
            // The heuristic information is the inverse of the cell distance to the goal
            heuristic = 1.0 / (euclideanDistance(i, j, mx, my) + 0.5);

            total_[i][j] = pow(pheromoneMatrix_[i][j], alpha_) * pow(heuristic, beta_);
        }
    }
}

/***** PATH PLANNING *****/
void ACO_v2::constructSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID) {

    unsigned int newCell = 0;
    vector<unsigned int> neighbours;
    //ROS_INFO("ANT STARTING POINT %d", startCellID);
    ant->setSearchTrue();
    //ROS_INFO("Ant %d is searching %d before construct",ant.getName(), ant.isSearching() );

    // Let the ant create the path
    while ( ant->getConstructedPath()->getPath().back() != goalCellID ){
        // Find free unvisited neighbours
        neighbours = getNeighbours(map, ant);
        //ROS_INFO("Neighbours cells %zu", neighbours.size());
        // Choose neighbour
        if( !neighbours.empty() ) {
            newCell = chooseNeighbour(map, neighbours);
            // Insert new cell
            ant->getConstructedPath()->insertCell(map, newCell);
            ant->setPosition(newCell);
            ant->addToVisited(newCell);
            //ROS_INFO("ADDED CELL %d", newCell);
        }
        else{ // If no neighbours have been found move the ant back of 5 positions. If this way it comes back to starting point delete it
            /* for( int i = 0; i < 5; i++ ){
                ant->removeVisited(ant->getConstructedPath()->getPath().rbegin()[1]);
                ant->getConstructedPath()->getPath().pop_back();
            }
            if ( ant->getConstructedPath()->getPath().empty() || ant->getConstructedPath()->getPath().back() == startCellID ) {
                ROS_INFO("ANT %d GOT STUCK", ant->getName());
                return;
            } */
            ant->getConstructedPath()->clearPath();
            ant->setPosition(startCellID);
            return;
        }
    }
    ant->setSearchFalse();
    //ROS_INFO("Ant %d is searching %d", ant.getName(), ant.isSearching() );
    //ROS_INFO("Ant %d found solution ", ant->getName());
}

vector<unsigned int> ACO_v2::computePaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID) {
    traceFile.open ("/home/fabio/catkin_ws/src/thesis_matherials/path_planning/ACO/ant_colony_path_planner/traceFile.txt");
    /* int notFree = 0;
    for(unsigned int i = 0; i < map->getHeight(); i++){
        for(unsigned int j = 0; j < map->getWidth(); j++){
            if(map->getCellCost(i, j) != 0) notFree++;
                //ROS_INFO("CELL %d %d free", i, j );
        }
    } */
    //ROS_INFO("ANT STARTING POINT %d", startCellID);

    // Number of iterations made. The termination condition for the external loop is reaching a certain number of iterations
    int iteration = 0;
    bestAntIndex = 0;
    int test = 0;
    ros::Time global = ros::Time::now();
    while( iteration < getIterationsLimit() ){
        // traceFile << "Iteration: " << iteration << endl;
        //ROS_INFO("Iteration %d", iteration);
        placeAnts(map, startCellID);
        //ROS_INFO("Placing ants");
        /*for( auto k: colony_ ){
            // Clear the path
            //ROS_INFO("Size colony %zu", colony_.size());
            ROS_INFO("VISITED PRIMA DI CLEAR %zu", k.getVisited().size());
            k->getConstructedPath()->clearPath();
            k->clearVisited();
            k->setSearchFalse();

            //ROS_INFO("CLEARED PATH LENGTH %zu, VISITED CLEARED %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
            // Place the ant at start cell
            k->getConstructedPath()->insertCell(map, startCellID);
            k->addToVisited(startCellID);
            k->setPosition(startCellID);
            k->setSearchTrue();

            //ROS_INFO("Formica %d piazzata in pos %d", k.getName(), k.getPosition());
            // ROS_INFO("START CELL AGGIUNTA %d", startCellID);
            //ROS_INFO("PATH AFTER FIRST CELL %zu cell inserted %d", k.getConstructedPath()->getPath().size(),  k.getConstructedPath()->getPath()[0]);
        }*/
        for(auto & i : colony_) {
            ros::Time start = ros::Time::now();
            // First construct solution for k ant
            //ROS_INFO("Ant %d is building a path", i->getName());
            for( auto j:  i->getVisited() ){
                if (j){
                    test++;
                }
            }
            //ROS_INFO("Ant %d already visited %d", i->getName(), test);
            test = 0;
            //ROS_INFO("ANT %d STARTING POINT %d", colony_[i].getName(), colony_[i].getConstructedPath()->getPath()[0]);
            constructSolutions(map, i, startCellID, goalCellID);
            //ROS_INFO("Ant %d is searching in compute paths %d", colony_[i].getName(), colony_[i].isSearching());
        }
        //ROS_INFO("test");
        // Update statistics
        updateStatistics(map, iteration);

        // Update pheromone trail
        //traceFile << "Map updated at iteration " << iteration << endl;
        pheromoneTrailUpdate(map, goalCellID);

        iteration++;
    }

    traceFile.close();
    ROS_INFO("Ant %d found shortest path with length %zu", bestAnt.getName(), bestAnt.getConstructedPath()->getPath().size());
    return bestAnt.getConstructedPath()->getPath();
}
