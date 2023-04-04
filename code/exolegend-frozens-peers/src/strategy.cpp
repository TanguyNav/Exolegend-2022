#include "gladiator.h"
#include <vector>
#include <iostream> 
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <map>
#include <string>


//using Path = std::vector<MazeSquare>;
using Path = std::vector<std::vector<uint8_t>>;

/*************************************
************** PLANNING **************
**************************************/

// Compute route to the closest Diamond in the arena with BFS
std::vector<Path> BFSPathToClosestDiamond(Gladiator* gladiator, MazeSquare source, int depth, int time, unsigned char ourId, unsigned char allyId, std::vector<unsigned char> ennemyIds) 
    {
        // Init resulting paths
        std::vector<Path> pathsToAllClosestDiamonds;
        std::vector<int> rewardForEachPath;

        // Test redefining source
        source = gladiator->maze->getNearestSquare();

        // Initial position of the Robot
        RobotData data = gladiator->robot->getData();
        Position position = data.position;
        //std::cerr << "Initial Robot Position: " << "x:" << position.x << " y:"<< position.y << " alpha angle:" << position.a << std::endl;
        //std::cerr << "Initial Robot Square: " << "(" << int(source.i) << " ,"<< int(source.j) << ")" << std::endl;

        // Position of other Robots
        RobotList robotList = gladiator->game->getPlayingRobotsId();
        // TODO: differentiate our robots from ennemies + get their positions

        // Init Data structures for computing BFS
        // TODO: REVENIR A LA PILE DE SQUARES
        int bestPath = 0;
        int bestReward = -1000;
        std::queue<std::pair<uint8_t, uint8_t>> q; // Pile of squares to visit
        std::map<std::pair<uint8_t, uint8_t>, std::pair<uint8_t, uint8_t>> discoveredBy; // <child, parent>
        std::map<std::pair<uint8_t, uint8_t>, int> pathLength;

        q.push(std::make_pair(uint8_t(source.i), uint8_t(source.j)));
        discoveredBy.emplace(std::make_pair(uint8_t(source.i), uint8_t(source.j)), std::make_pair(uint8_t(-1), uint8_t(-1)));
        pathLength.emplace(std::make_pair(uint8_t(source.i), uint8_t(source.j)), 0);
        gladiator->log("INIT PATH FINDING");

        // Storing coordinates for North, West, South, East 
        std::vector<std::pair <int, int>> neighboorSquaresCoords = {std::make_pair(0,1), std::make_pair(-1,0), std::make_pair(0,-1), std::make_pair(1,0)};


        // Constant for grid
        float squareSize = gladiator->maze->getSquareSize();
        float epsilonDistToRobot = 0.3;


        bool skip = false;
                
        while (!q.empty()) {
            skip = false;

            // Get current square data
            std::pair <uint8_t, uint8_t> currCoords = q.front(); q.pop();
            MazeSquare currSquare = gladiator->maze->getSquare(currCoords.first, currCoords.second);
            //std::cerr << "Square (" << int(currCoords.first) << "," << int(currCoords.second) << ")" << std::endl;
            Coin coin = currSquare.coin;
            int currDepth = pathLength[std::make_pair(uint8_t(currSquare.i), uint8_t(currSquare.j))];
            //std::cerr << "currDepth :" << currDepth << std::endl;

            /**
            * TODO: Manage having another robot on a Tile
            **/ 
            // For instance:
                // Do not come close to an ally
                // Choose if we try to attack an ennemy or avoid it
            /*
            if (robot on currSquare) {
                if (robot is mine) {
                    do something
                }
                else {
                    do somethign else
                }
            }
            */

            // Avoid if a path is too close to an ally
            /*
            RobotData data = gladiator->game->getOtherRobotData(allyId);
            Position position = data.position;
            float x = (currCoords.first + 0.5) * squareSize;
            float y = (currCoords.second + 0.5) * squareSize; 

            float dx = x - position.x
            float dy = y - position.y
            float euclidean_dist = dx * dx + dy * dy;
            // Avoid if too close to other robot
            if (euclidean_dist < epsilonDistToRobot) 
            {
                    continue;                
            }
           */
        
           /*
           for (int i=0; i < ennemyIds.size())
           {
                RobotData data = gladiator->game->getOtherRobotData(ennemyIds.ids[i]);
                Position position = data.position;
                float x = (currCoords.first + 0.5) * squareSize;
                float y = (currCoords.second + 0.5) * squareSize; 

                float dx = x - position.x
                float dy = y - position.y
                float euclidean_dist = dx * dx + dy * dy;
                // Avoid if too close to other robot
                if (euclidean_dist < epsilonDistToRobot) 
                {
                        continue;                
                }
           }
           */



            /**
            * Find Diamonds *slurp*:
            **/ 

            // TODO: Add a condition if current diamond may be claimed by another Robot (friend of ennemy)
            // Stop travelling where there is a diamond on a square
            
            if(currDepth == depth) {

                // Computing current path
                Path intermediatePath;

                // Variables for Diamond bonus
                int diamondScore = 0;

                // Variables for rotation penalty
                int rotationPenalty = 0;
                int prevXGap = -10;
                int prevYGap = -10;

                // Variables for close to border penalty
                int closingBorderPenalty = 0;
                int borderStep = 2;
                int removedBorderTiles = (int)time / 20;

                //gladiator->log("Coin found, CuurCoord : (%d, %d)", currCoords.first, currCoords.second);
                skip = true;
                while (!((currCoords.first == source.i) && (currCoords.second == source.j))) 
                {   

                    // For computing rotation penalty
                    int prev_x = currCoords.first;
                    int prev_y = currCoords.second;

                    // Diamond bonus
                    diamondScore += currSquare.coin.value;
                    
                    // Border penalty
                    if (prev_x <= removedBorderTiles + borderStep) 
                    {
                        closingBorderPenalty += removedBorderTiles + borderStep - prev_x;
                    }
                    if (prev_x >= 14 - 1 - removedBorderTiles - borderStep)
                    {
                        closingBorderPenalty += prev_x - (14 - 1 - removedBorderTiles - borderStep);
                    }
                    if (prev_y <= removedBorderTiles + borderStep)
                    {
                        closingBorderPenalty += removedBorderTiles + borderStep - prev_y;
                    }
                    if (prev_y >= 14 - 1 - removedBorderTiles - borderStep)
                    {
                        closingBorderPenalty += prev_y - (14 - 1 - removedBorderTiles - borderStep);
                    }   
                    
                    
                    // Smaller penalty when small square
                    if (removedBorderTiles > 3)
                    {
                        if (closingBorderPenalty > 0)
                        {
                            closingBorderPenalty = 5;
                        }
                    }
                    
                                

                    std::vector<uint8_t> currCoordsVec = {currCoords.first, currCoords.second};
                    intermediatePath.push_back(currCoordsVec);
                    //std::cerr << "Pushed back " << int(currSquare->i) << " and " << int(currSquare->j) << std::endl;
                    currCoords = discoveredBy.at(currCoords);
                    currSquare = gladiator->maze->getSquare(currCoords.first, currCoords.second);
                    //coin.value = 1;
                    //gladiator->log("whillllllle");

                    /**
                    * Compute Reward
                    **/

                    // Penalty on rotations
                    int x_gap = currCoords.first - prev_x;
                    int y_gap = currCoords.second - prev_y;
                    if (prevXGap != -10) {
                        if ((prevXGap != x_gap) || ( prevYGap != y_gap)) {
                            rotationPenalty += 1;
                        }
                    } 
                    prevXGap = x_gap;
                    prevYGap = y_gap;

                    

                }

                // REWARD
                //std::cerr << "closingBorderPenalty " << closingBorderPenalty << std::endl;
                int intermediateReward = 10*diamondScore - 4*rotationPenalty - closingBorderPenalty;

                if (intermediatePath.size() != 0)
                {
                    std::reverse(intermediatePath.begin(), intermediatePath.end());
                    pathsToAllClosestDiamonds.push_back(move(intermediatePath));
                    //gladiator->log("Path trouve : (%d)", pathsToAllClosestDiamonds.size());
                    if (intermediateReward > bestReward)
                    {
                        bestReward = intermediateReward;
                        bestPath =  pathsToAllClosestDiamonds.size() -1;
                    }
                    //gladiator->log("bestPath: %d", bestPath);
                }
            }
            
            // Only add neighbours of squares without diamond (so we don't go beyond that limit).
            if (!skip)
            {
                if (currDepth < depth || ((currCoords.first == source.i) && (currCoords.second == source.j))) {
                    // Test neighboors tiles
                    std::vector<MazeSquare*> neighboorSquares = {currSquare.northSquare, currSquare.westSquare, currSquare.southSquare, currSquare.eastSquare};
                    //gladiator->log("START COIN VALUE, CuurCoord : (%d, %d)", currCoords.first, currCoords.second);
                    bool foundValuableSquare = true; // For storing if there is any admissible square
                    int countSurroundingWalls = 0; 

                    for (unsigned int i=0; i < 4; i++) 
                    {
                        if (neighboorSquares[i] == NULL) {
                            countSurroundingWalls += 1;
                            // TODO: differentiate square management when there is a wall
                            gladiator->log("There is a wall on this tested neighboor square: (%d,%d):", neighboorSquaresCoords[i].first, neighboorSquaresCoords[i].second);
                        } else {
                            if (discoveredBy.count(std::make_pair(int(neighboorSquares[i]->i), int(neighboorSquares[i]->j)))==0) 
                            {
                            foundValuableSquare = false;
                            
                            pathLength[std::make_pair(int(neighboorSquares[i]->i), int(neighboorSquares[i]->j))] = pathLength.at(currCoords) + 1;
                            
                            discoveredBy.emplace(std::make_pair(int(neighboorSquares[i]->i), int(neighboorSquares[i]->j)), currCoords);
                            //gladiator->log("discovered tile: (%d,%d):", neighboorSquaresCoords[i].first, neighboorSquaresCoords[i].second);

                            q.push(std::make_pair(int(neighboorSquares[i]->i), int(neighboorSquares[i]->j)));
                            
                            } else {
                                gladiator->log("The tile was already discovered: (%d,%d):", neighboorSquaresCoords[i].first, neighboorSquaresCoords[i].second);
                            }
                        }                        
                    }

                    // Manage corridor
                    if (countSurroundingWalls == 3)
                    {
                        for (unsigned int i=0; i < 4; i++) 
                        {
                            if (neighboorSquares[i] == NULL) {

                                int neighboorCoordX = currCoords.first + neighboorSquaresCoords[i].first;
                                int neighboorCoordY = currCoords.second + neighboorSquaresCoords[i].second;

                                if (discoveredBy.count(std::make_pair(neighboorCoordX, neighboorCoordY))==0) 
                                {
                                    foundValuableSquare = false;
                                    
                                    pathLength[std::make_pair(neighboorCoordX, neighboorCoordY)] = pathLength.at(currCoords) + 1;
                                    
                                    discoveredBy.emplace(std::make_pair(neighboorCoordX, neighboorCoordY), currCoords);
                                    //gladiator->log("discovered tile: (%d,%d):", neighboorSquaresCoords[i].first, neighboorSquaresCoords[i].second);

                                    q.push(std::make_pair(neighboorCoordX, neighboorCoordY));
                                
                                } 

                            }
                        }
                    }

                     // Manage the end of a corridor
                    if (!foundValuableSquare) 
                    {
                        pathLength[std::make_pair(int(currCoords.first), int(currCoords.second))] = depth;
                    }

                }
            }

        }
        //gladiator->log("Path retoune : (%d)", pathsToAllClosestDiamonds.size());

        // Get best path
        std::vector<Path> bestPathToDiamonds;
        bestPathToDiamonds.push_back(pathsToAllClosestDiamonds[bestPath]);
        
        return bestPathToDiamonds;
    }



void displayPaths(Gladiator* gladiator, std::vector<Path> pathsToDisplay) 
{
    //gladiator->log("Number of paths: " + String(pathsToDisplay.size()));
    std::cerr << "Number of paths: " << pathsToDisplay.size() << std::endl;
    
    // Display coordinates of successives squares for each path
    for (unsigned int k = 0; k < pathsToDisplay.size(); k++) 
    {
        //gladiator->log("Path number " + String(k));
        std::cerr << "Path number " << k << std::endl;
        for (unsigned int l = 0; l < pathsToDisplay[k].size(); l++) 
        {
            //unsigned int i = pathsToDisplay[k][l][0]; 
            //unsigned int j = pathsToDisplay[k][l][1];
            //gladiator->log(String("Tile (") + String(i) + "," + String(j) + ")");
            std::cerr << "Square (" << int(pathsToDisplay[k][l][0]) << "," << int(pathsToDisplay[k][l][1]) << ")" << std::endl;
        }

    }
}


/*************************************
************** Servoing **************
**************************************/
void servoing(Gladiator* gladiator,  MazeSquare targetSquare)
{
    // The math are here: https://www.pm-robotix.eu/2022/02/02/asservissement-et-pilotage-de-robot-autonome/

    /**
     * Retrieving usefull data from context 
    **/

    // Get const Data from Robot
    const float WheelRadius = gladiator->robot->getWheelRadius();
    const float RobotRadius = gladiator->robot->getRobotRadius();

    // Get const Data from Maze
    float squareSize = gladiator->maze->getSquareSize(); 

    // Get current Robot Data
    RobotData data = gladiator->robot->getData();
    Position position = data.position; // Location of the robot
    //double vl = data.vl; // Left wheel speed
    //double vr = data.vr; // Right wheel speed
    //double speedLimit = data.speedLimit;

    /**
     * Compute next position to reach 
    **/

    // Get aimed coordinates
    // For now, we take central coordinates of the aimed square as objective
    Position centerCoor;
    centerCoor.x = (targetSquare.i + 0.5) * squareSize;
    centerCoor.y = (targetSquare.j + 0.5) * squareSize;   

    // Compute distance and angle to next goal
    float x_disp = centerCoor.x - position.x;
    float y_disp = centerCoor.y - position.y;

    float dot = 1.0 * x_disp; // dot product between [1, 0] and [x_disp, y_disp]
    float det = 1.0 * y_disp; // determinant
    float dispAngle = atan2(det, dot) - position.a; // Angular distance to travel around robot mass center axis


    /** 
    * Servoing: For now, we consider rotation and translation components separately 
    **/ 

    // Rotation components
    float dL = dispAngle*(WheelRadius-RobotRadius/2.0f); // Left
    float dR = dispAngle*(WheelRadius+RobotRadius/2.0f); // Right
    
    // Take dynamics into account as we can not apply the same acceleration on the wheel at the same time
    // TODO: Recompute the following values with our weapon
    float vMax = 0.8; // In m/s
    float accMax = 1.0; // In m/s
    float decMax = 1.0; // In m/s

    float ratio = dR/dL;
    float vMaxL, accL, decL;
    float vMaxR, accR, decR;
    if(fabs(dL) > fabs(dR))
    {
        vMaxL = vMax; accL = accMax; decL = decMax;
        vMaxR = vMax*ratio; accR = accMax*ratio; decR = decMax*ratio;
    }
    else
    {
        vMaxL = vMax*ratio; accL = accMax*ratio; decL = decMax*ratio;
        vMaxR = vMax; accR = accMax; decR = decMax;
    }

    std::cerr << "vMaxL:" << vMaxL << ", accL:" << accL << ", decL:" << decL << std::endl;
    std::cerr << "vMaxR:" << vMaxR << ", accR:" << accR << ", decR:" << decR << std::endl;

    // Linear component
    // TODO
        
        
        // vitesse du robot
        

}

