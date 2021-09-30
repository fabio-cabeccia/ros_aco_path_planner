//
// Created by fabio on 09/09/2021.
//

#include "ACO.h"
#include "Ant.h"
#include "OccupancyGrid.h"
#include "Path.h"

#include <ros/ros.h>

#include <cmath>
#include <random>
#include <vector>


int goalFlag = 0;
using namespace std;
/***** CONSTRUCTORS/DESTRUCTORS *****/
ACO::ACO() = default;

ACO::ACO(double alpha, double beta, double evaporationRate, double initialPheromoneValue, int antNumber,int iterationsLimit) {
    setAlpha(alpha);
    setBeta(beta);
    setEvaporationRate(evaporationRate);
    setInitialPheromoneValue(initialPheromoneValue);
    setAntNumber(antNumber);
    setIterationsLimit(iterationsLimit);
    vector<Ant* > initCol(antNumber);
    colony = initCol;
    bestIteration = 0;
    pheromoneMatrix_.clear();
    total_.clear();
}

ACO::~ACO(void) = default;

/***** MUTATORS *****/
void ACO::setPheromoneMatrix(vector<vector<double>> pheromoneMatrix) {
    pheromoneMatrix_ = pheromoneMatrix;
}
void ACO::setTotalMatrix(vector<vector<double>> total) {
    total_ = total;
}
void ACO::setAlpha(double alpha) {
    alpha_ = alpha;
}
void ACO::setBeta(double beta) {
    beta_ = beta;
}
void ACO::setEvaporationRate(double evaporationRate) {
    evaporationRate_ = evaporationRate;
}
void ACO::setInitialPheromoneValue(double initialPheromoneValue) {
    initialPheromoneValue_ = initialPheromoneValue;
}
void ACO::setAntNumber(int antNumber) {
    antNumber_ = antNumber;
}
void ACO::setIterationsLimit(int it) {
    iterationsLimit_ = it;
}

vector<vector<double>> ACO::getPheromoneMatrix() {
    return pheromoneMatrix_;
}
vector<vector<double>> ACO::getTotal(){
    return total_;
}
double ACO::getAlpha() {
    return alpha_;
}
double ACO::getBeta() {
    return beta_;
}
double ACO::getEvaporationRate() {
    return evaporationRate_;
}
double ACO::getInitialPheromoneValue() {
    return initialPheromoneValue_;
}
int ACO::getAntNumber() {
    return antNumber_;
}
int ACO::getIterationsLimit() {
    return iterationsLimit_;
}

/***** INITIALIZATION *****/
void ACO::initializePheromoneMatrix(OccupancyGridMap* map) {
    ROS_INFO("Initializing pheromone matrix.");
    //traceFile << "Initial pheromone matrix" << endl ;
    vector<vector<double>> init(map->getHeight(), vector<double>(map->getWidth()));
    pheromoneMatrix_ = init;

    for (int i = 0; i < map->getHeight(); i++) {

        for (int j = 0; j < map->getWidth(); j++) {

            if (map->isFree(i, j)) {
                pheromoneMatrix_[i][j] = getInitialPheromoneValue();
                //traceFile << getInitialPheromoneValue() << " ";
            } else if (map->isObstacle(i, j)){
                pheromoneMatrix_[i][j] = 0.0; // FLT_MAX indicates an obstacle
                //traceFile << FLT_MAX << " ";
            }
            else{
                pheromoneMatrix_[i][j] = getInitialPheromoneValue()/map->getCellCost(i, j);
            }
        }
        //traceFile << endl;
    }
}
void ACO::initializeTotalMatrix(OccupancyGridMap* map) {
    ROS_INFO("Initializing pheromone + heuristic matrix.");
    traceFile << "Initial total matrix" << endl ;
    vector<vector<double>> init( map->getHeight(), vector<double>(map->getWidth()) );
    total_ = init;

    computeTotalInformation(map);
}

/***** UTILITIES *****/
vector<unsigned int> ACO::getNeighbours(OccupancyGridMap* map, unsigned int cellID) {
    unsigned int mx = 0;
    unsigned int my = 0;
    unsigned int debugIndex = 0;
    map->indexToCells(cellID, mx, my);
    vector<unsigned int> neighbourCells;

    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++) {
            //check whether the index is valid
            if ( (mx + i > 0) &&  (mx + i < map->getHeight()) && (my + j > 0) && (my + j < map->getWidth())) {
                neighbourCells.emplace_back(map->getCellIndex(mx + i, my + j));
            }
        }
    return neighbourCells;
}
unsigned int ACO::chooseNeighbour(OccupancyGridMap* map, Ant* ant) {
    // Define a probability vector. Its size depends on the amount of neighbours
    vector<pair<unsigned int, double>> probabilities;
    // Get neighbours of current ant position
    vector<unsigned int> neighbours = getNeighbours(map, ant->getPosition());
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

    for( auto i: neighbours ){ // For each neighbour cell check if free

        if(map->isFree(i)){ // If free check whether is already visited
            if(!isVisited(ant, i)){
                map->indexToCells(i, mx, my);
                probabilities.emplace_back(i, total_[mx][my]); // Cell index, total computed cost of the cell
                probSum += probabilities.back().second;
                //ROS_INFO("Probsum %.1f", probSum);
            }
            else {
                probabilities.emplace_back(i, 0.0);
            }
        }
        // ROS_INFO("Ant %d, TIMES %d, VALID %d, VISITED %d", ant->getName(), l, map->isValid(i), isVisited(ant, i));
        l++;
    }
    mx = my = 0;
    // Choose the neighbour
    //ROS_INFO("Probsum %.1f", probSum);
    if( probSum <= 0 ){ // No feasible neighbour cells.
        // ROS_INFO("STUCK");
        /* // Delete the last element of the path since it leads to a deadlock
        ant->getConstructedPath()->popBack();
        // Update the cost
        ant->getConstructedPath()->computeCost(map);
        // Update the ant position
        ant->setPosition(ant->getConstructedPath()->getPath().back());
        // Recursive call with updated ant position, path cells and path cost.
        chosen = chooseBestNext(map, ant); */
        chosen = chooseBestNext(map, ant);
    }
    else{ // At least one of the neighbours is eligible. Select one according to the probabilities rule.
        // Select a random value in the interval [0, probSum)
        double random = getRandom(0, probSum);
        // Index for updating partialSum
        int i = 0;
        // Set the initial value for partial sum
        partialSum += probabilities[0].second;
        while ( partialSum <= random && i < neighbours.size()){
           i++;
           partialSum += probabilities[i].second;
        }
        if ( i == neighbours.size()) {
            chosen = chooseBestNext(map, ant);
        }
        else{
            // ROS_INFO("CHOSEN NORMALMENTE");
            chosen = probabilities[i].first;
        }
        i = 0;
        partialSum = 0;
    }

    return chosen;
}
bool ACO::isVisited(Ant* ant, unsigned int cellID) {
    for (auto i: ant->getVisited()) {
        if (i == cellID)
            return true;
    }
    return false;
}
void ACO::updateStatistics(OccupancyGridMap* map, int iteration) {
    unsigned int indexBestOfIteration = findBestAnt();

    if (colony[bestAntIndex]->getConstructedPath()->getCost() == 0.0 ||
        colony[bestAntIndex]->getConstructedPath()->getCost() >
        colony[indexBestOfIteration]->getConstructedPath()->getCost()) {
        /* for( auto l: colony[indexBestOfIteration]->getConstructedPath()->getPath() ){
            bestAnt->getConstructedPath()->insertCell(map, l);
        }
        bestAnt->setPosition(colony[indexBestOfIteration]->getPosition());
        bestAnt->setName(colony[indexBestOfIteration]->getName());
        bestIteration = iteration; */
        bestAntIndex = indexBestOfIteration;
    }

    //ROS_INFO("Iteration %d - Best Ant of iteration: %d, Path length: %.2f", iteration, indexBestOfIteration,
             // colony[indexBestOfIteration]->getConstructedPath()->getCost());
    ROS_INFO("Current best ant: %d, Path cost: %.1f, Path length: %zu", colony[bestAntIndex]->getName(), colony[bestAntIndex]->getConstructedPath()->getCost(), colony[bestAntIndex]->getConstructedPath()->getPath().size());
    /* traceFile << "Current Iteration " << iteration << "- Best ant of iteration: " << indexBestOfIteration
              << ", Path length: " << colony[indexBestOfIteration]->getConstructedPath()->getCost() << endl;
    traceFile << "Iteration of best ant: " << bestIteration << "- Best ant: " << bestAnt->getName() << ", Path length: "
              << bestAnt->getConstructedPath()->getCost() << endl; */
}
int ACO::findBestAnt() {
    Path* bestPath = colony[0]->getConstructedPath();
    int best = 0;
    for(int k = 0; k < colony.size(); k++){
        if ( bestPath->getCost() > colony[k]->getConstructedPath()->getCost() && colony[k]->getConstructedPath()->getCost() != 0) {
            best = k;
            bestPath = colony[k]->getConstructedPath();
            //ROS_INFO("NEW BEST %.1f, SIZE %zu", bestPath->getCost(), bestPath->getPath().size());
        }
    }
    return best;
}
double ACO::getRandom(double minBound, double maxBound){
    random_device rd;
    default_random_engine rdGenerator( rd() );
    uniform_real_distribution<> rdValue(minBound, maxBound);

    return rdValue(rdGenerator);
}
bool ACO::endSearchCondition(unsigned int goalCellID, int searchCondition){
    while( searchCondition <= 10000 ) {
        for (auto k: colony) {
            if (k->getPosition() != goalCellID)
                return false;
        }
    }
    return true;
}
unsigned int ACO::chooseBestNext(OccupancyGridMap* map,Ant* ant){
    double value_best = -1.;
    unsigned int nxtCell = 0;
    unsigned int mx = 0, my = 0;
    int cnt = 0;
    vector<unsigned int> neighbours = getNeighbours(map, ant->getPosition());

    for( auto i: neighbours ){
        if( map->isFree(i) ){
            // ROS_INFO("NEIGHBOURS %d in choseBestNext is Free", i);
            map->indexToCells(i, mx, my);
            if( value_best < total_[mx][my] ){
                value_best = total_[mx][my];
                nxtCell = i;
            }
        }
        else
            cnt++;
    }
    if( cnt == neighbours.size() ){
        return -1;
    }
    return nxtCell;
}
void ACO::placeAnts(OccupancyGridMap* map, unsigned int startCellID){
    ROS_INFO("Placing ants");
    for( auto k: colony ){
        // Clear the path
        // ROS_INFO("VISITED PRIMA DI CLEAR %zu", k->getVisited().size());
        k->getConstructedPath()->clearPath();
        k->clearVisited();
        k->getConstructedPath()->setCost(0);
        // ROS_INFO("VISITED DOPO DI CLEAR %zu", k->getVisited().size());

        // ROS_INFO("CLEARED PATH LENGTH %zu, VISITED CLEARED %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
        // Place the ant at start cell
        k->getConstructedPath()->insertCell(map, startCellID);
        k->setPosition(startCellID);
        k->addToVisited(startCellID);
        // ROS_INFO("START CELL AGGIUNTA %d", startCellID);
        // ROS_INFO("PATH AFTER FIRST CELL %zu, VISITED AFTER FIRST CELL %zu", k->getConstructedPath()->getPath().size(),  k->getVisited().size());
    }
}

/***** PHEROMONE + HEURISTIC *****/
void ACO::pheromoneTrailUpdate(OccupancyGridMap* map) {
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
    computeTotalInformation(map);
}
void ACO::asUpdate(OccupancyGridMap* map) {
    for( auto k: colony ){
        globalPheromoneUpdate(map, k);
    }
}
void ACO::globalPheromoneUpdate(OccupancyGridMap* map, Ant* ant) {
    // Cells coordinates
    unsigned int mx = 0;
    unsigned int my = 0;
    // Reinforce only the edges used in ant's solution
    double d_tau = 1.0 / ant->getConstructedPath()->getCost();

    for( unsigned int i: ant->getConstructedPath()->getPath() ) {
        // Update mx and my
        map->indexToCells(i, mx, my);
        pheromoneMatrix_[mx][my] += d_tau;
    }
}
void ACO::computeTotalInformation(OccupancyGridMap* map) {
    // mx & my
    for (unsigned int i = 0; i < map->getHeight(); i++) {
        for (unsigned int j = 0; j < map->getWidth(); j++) {
            // The heuristic information is the inverse of the cell cost.
            double heuristic = 1.0 / (map->getCellCost(i, j) +
                                      0.5); // Add a little constant so there is not a division by 0 if the cell cost is zero
            total_[i][j] = pow(pheromoneMatrix_[i][j], alpha_) * pow(heuristic, beta_);
        }
    }
}

/***** PATH PLANNING *****/
void ACO::constructSolutions(OccupancyGridMap* map, Ant* ant, unsigned int startCellID, unsigned int goalCellID) {
    ROS_INFO("Ant %d is building a path", ant->getName());
    unsigned int newCell = 0;
    // Let the ant create the paths
    ROS_INFO("ANT STARTING POINT %d", ant->getPosition());
    while ( ant->getPosition() != goalCellID ){
        // Choose neighbour and move in it
        newCell = chooseNeighbour(map, ant);
        ant->getConstructedPath()->insertCell(map, newCell);
        ant->setPosition(newCell);
        ant->addToVisited(newCell);
        //ROS_INFO("ADDED CELL %d", newCell);
    }
    ROS_INFO("ANT %d found solution ", ant->getName());
}
void ACO::constructParallelSolutions(OccupancyGridMap* map, unsigned int goalCellID) {
    unsigned int mx = 0, my = 0, ant1 = 0, ant2 = 0, goal1 = 0, goal2 = 0;
    map->indexToCells(goalCellID, goal1, goal2);

    for( auto k: colony ){
        if( k->getPosition() != goalCellID ){ // If k ant has already reached the goal it won't move any further
            // Choose next cell
            unsigned int newCell = chooseNeighbour(map, k);
            map->indexToCells(newCell, mx, my);
            map->indexToCells(k->getPosition(), ant1, ant2);
            // ROS_INFO("Ant %d moved from (%d, %d) to (%d, %d). Goal position (%d, %d).", k->getName(), ant1, ant2, mx, my, goal1, goal2);
            k->getConstructedPath()->insertCell(map, newCell);
            k->setPosition(newCell);
            k->addToVisited(newCell);
            if(k->getPosition() == goalCellID )
                goalFlag++;
        }
    }
}
Path *ACO::computePaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID) {
    traceFile.open ("/home/fabio/catkin_ws/src/thesis_matherials/path_planning/ACO/ant_colony_path_planner/traceFile.txt");
    for(unsigned int i = 0; i < map->getHeight(); i++){
        for(unsigned int j = 0; j < map->getWidth(); j++){
            if(map->getCellCost(i, j) == 0)
                ROS_INFO("CELL %d %d free", i, j );
        }
    }
    // Initialize all ACO components
    initializePheromoneMatrix(map);
    initializeTotalMatrix(map);
    // Colony init
    for( int i = 0; i < getAntNumber(); i++){
        colony[i] = new Ant();
        colony[i]->setName(i);
    }
    // Number of iterations made. The termination condition for the external loop is reaching a certain number of iterations
    int iteration = 0;
    bestAntIndex = 0;
    ros::Time global = ros::Time::now();
    while( iteration < getIterationsLimit() ){
        // traceFile << "Iteration: " << iteration << endl;
        // ROS_INFO("Iteration %d", iteration);
        placeAnts(map, startCellID);
        for(auto k : colony) {
            ros::Time start = ros::Time::now();
            // First construct solution for k ant
            constructSolutions(map, k, startCellID, goalCellID);
            // Then Local search
            // localSearch();
        }
        // Update statistics
        updateStatistics(map, iteration);
        // Update pheromone trail
        //traceFile << "Map updated at iteration " << iteration << endl;
        pheromoneTrailUpdate(map);
        iteration++;
    }

    traceFile.close();
    ROS_INFO("Best ant %d path length %.1f", colony[bestAntIndex]->getName(), colony[bestAntIndex]->getConstructedPath()->getCost());
    return colony[bestAntIndex]->getConstructedPath();
}
Path* ACO::computeParallelPaths(OccupancyGridMap* map, unsigned int startCellID, unsigned int goalCellID){
    traceFile.open ("/home/fabio/catkin_ws/src/thesis_matherials/path_planning/ACO/ant_colony_path_planner/traceFile.txt");
    unsigned int mxs = 0, mys = 0, mxg = 0, myg = 0;
    map->indexToCells(startCellID, mxs, mys);
    map->indexToCells(goalCellID, mxg, myg);

    ROS_INFO("Start cell = %d, %d, Start index = %d Goal = %d, %d, Goal index = %d", mxs, mys, startCellID, mxg, myg, goalCellID);
    // Initialize all ACO components
    initializePheromoneMatrix(map);
    initializeTotalMatrix(map);
    unsigned int index = map->getCellIndex(mxs, mys);
    ROS_INFO("Index of start in total %d", index);
    // Colony init
    for( int i = 0; i < getAntNumber(); i++){
        colony[i] = new Ant();
        colony[i]->setName(i);
    }
    // Number of iterations made. The termination condition for the external loop is reaching a certain number of iterations
    int iteration = 0;
    ros::Time global = ros::Time::now();
    while( iteration < getIterationsLimit() ){
        // traceFile << "Iteration: " << iteration << endl;
        ROS_INFO("Iteration %d", iteration);
        placeAnts(map, startCellID);
        int searchLimit = 0;
        while( !endSearchCondition(goalCellID, searchLimit) ) { // All ants must have reached the goal
            //ROS_INFO("ANTS THAT FOUND SOLUTION %d. CYCLE NUMBER: %d", goalFlag, searchLimit);
            ros::Time start = ros::Time::now();
            // Every ant will construct its own solution in parallel to the others, which means that every iteration avery ant will move
            // in a new cell.
            constructParallelSolutions(map, goalCellID);
            // Then Local search
            // localSearch();
            // Update statistics
            updateStatistics(map, iteration);
            // Update pheromone trail
            //traceFile << "Map updated at iteration " << iteration << endl;
            pheromoneTrailUpdate(map);
            searchLimit++;
        }

        iteration++;
    }

    traceFile.close();
    return bestAnt->getConstructedPath();
}

/***** LOCAL SEARCH *****/
/* void ACO::localSearch(){
    for ( auto k:  colony ) {
        twoOptFirst( k->getConstructedPath() );
        k = compute_tour_length( ant[k].tour );
        if (termination_condition()) return;
    }
} */
void ACO::twoOptFirst(Path* path) {

}





