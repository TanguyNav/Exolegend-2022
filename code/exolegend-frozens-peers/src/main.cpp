#include "gladiator.h"
#include "strategy.h"
#include <chrono>

// Problem variables
#define GRID_SIZE 14

// TEST VARIABLE
#define FREE 0

// The state machine
#define INIT 0
#define DESIRED_POSITION 1
#define ANGLE_AND_DISTANCE 2
#define ANGLE_FINAL 3

// Path variable
#define DEPTH 4

Gladiator *gladiator;
unsigned char ourId;
unsigned char allyId;
std::vector<unsigned char> ennemyIds;

// Servoing variables
float WheelRadius;
float RobotRadius;
float squareSize; 
int etat;
float speedG;
float speedD;
Position posd;
Position pos0;
float thd;
float thd_final;
float distd;
float erra;
float errd;
bool backward;

// Timer variable
bool isChronoInit;
std::chrono::time_point<std::chrono::system_clock> startTime, endTime;

// Global variables for the planning
bool hasObjective;
Path curr_route;
int step;
int maxStep;

bool isMazeStarted;
MazeSquare startingSquare;


void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);

    if (FREE == 1)
    {
        Position initPosition = {12, 12, 1.52};
        gladiator->game->enableFreeMode(RemoteMode::OFF, initPosition);
    } 
    
}

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match


    // Get team data
    RobotData ourRobotData = gladiator->robot->getData();
    ourId = ourRobotData.id;
    unsigned char ourteamId = ourRobotData.teamId;
    ennemyIds.clear();
    RobotList robotList = gladiator->game->getPlayingRobotsId();
    for(uint8_t i = 0; i < 4; i++)
    {
        if (robotList.ids[i] != ourId)
        {
            RobotData data = gladiator->game->getOtherRobotData(robotList.ids[i]);
            // Our ally
            if (data.teamId == ourteamId)
            {
                allyId = data.id;
                gladiator->log("Our ally id %d", allyId);
            } else {
                ennemyIds.push_back(data.id);
                gladiator->log("Other player id %d", data.id);
            }

        } else {
            gladiator->log("Our robot with id %d", ourId);
        }   
    }

    // Chrono variable
    isChronoInit = false;

    // Variables for the planning
    hasObjective = false;
    step = 0;

    // Servoing variables
    WheelRadius = gladiator->robot->getWheelRadius();
    RobotRadius = gladiator->robot->getRobotRadius();
    squareSize = gladiator->maze->getSquareSize(); 
    etat = INIT;
    speedG = 0.0;
    speedD = 0.0;
    thd = 0.0;
    thd_final = 0.0;
    distd = 0.0;
    erra = 0.0;
    errd = 0.0;
    backward = false;

    // Init PID corrector coefficients
    /**/
    float kp = 1; // proportional coefficient
    float ki = 5; // integral coefficient
    float kd = 0; // derivative coefficient
    gladiator->control->setWheelPidCoefs(WheelAxis::RIGHT, kp, ki, kd);
    gladiator->control->setWheelPidCoefs(WheelAxis::LEFT, kp, ki, kd);
    /**/

    // Init starting square
    isMazeStarted = false;

}

void loop()
{
    if (gladiator->game->isStarted())
    { 
        // Init chrono if match started
        if (!isChronoInit)
        {
            startTime = std::chrono::system_clock::now();
            isChronoInit = true;
        }

        // Init starting Maze Square
        if (!isMazeStarted)
        {
            startingSquare = gladiator->maze->getNearestSquare();
            isMazeStarted = true;
        }
        

        // Get position of other Robots
        
        // Update or init shortest path from the gladiator to the closest diamond
        // The distance measure is the number of crossed squares
        if (FREE == 0)
        {
            
            if (!hasObjective)
            {
                endTime = std::chrono::system_clock::now();
                int time = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
                std::vector<Path> pathsToAllClosestDiamonds = BFSPathToClosestDiamond(gladiator, startingSquare, DEPTH, time, ourId, allyId, ennemyIds);
                //displayPaths(gladiator, pathsToAllClosestDiamonds);
                hasObjective = true; // To know we don't need to comptue a new route as we already have one
                curr_route = pathsToAllClosestDiamonds[0]; //For now we choose the first path by default
                step = 0; // Route step
                //std::cerr << "Square (" << int(curr_route[step][0]) << "," << int(curr_route[step][1]) << ")" << std::endl;
                maxStep = curr_route.size(); // Number of route steps
                //std::cerr << "Curr route size" << int(maxStep) << std::endl;

                // Elimination of intermediate target points on the same line
                gladiator->log("maxStep: %i",int(maxStep));
                if(maxStep>2){

                    // Suitable for Depth = 3 only
                    if(curr_route[2][0]==curr_route[1][0] && curr_route[1][0]==curr_route[0][0] ||(curr_route[2][1]==curr_route[1][1] && curr_route[1][1]==curr_route[0][1])){
                        // && curr_route[k][0]==curr_route[k-1][0]
                        // ||(curr_route[k+1][1]==curr_route[k][1] && curr_route[k][1]==curr_route[k-1][1])
                        curr_route[1][0]=curr_route[2][0];
                        curr_route[1][1]=curr_route[2][1];
                    }


                }
            
            }

        // FREE MODE
        } else {
            // Simu START IS (0,7)
            std::vector<std::vector<uint8_t>> homemade_route;

            // Here define the successive coordinates of the squared maze to reach
            std::vector<uint8_t> curr_coords = {0, 8};
            homemade_route.push_back(curr_coords);

            // Global variables
            curr_route = homemade_route;
            hasObjective = true;
            step = 0; // Route step
            maxStep = curr_route.size();
        }
        
        // Shitty debug code
        /*if (hasObjective) 
        {
            std::cerr << "int(curr_route[step][0]):" << int(curr_route[step][0]) << std::endl;
            std::cerr << "int(curr_route[step][1]:" << int(curr_route[step][1]) << std::endl;
        } 
        
        /*
        if (int(curr_route[step][0]) < 0 || int(curr_route[step][1] < 0)) 
        // || (int(curr_route[step][0]) >= 14) || (int(curr_route[step][1]) >= 14)
        {
            std::cerr << "TEST:" << std::endl;
        }  */


        // Servoing
        // std::cerr << "TEST:" << curr_route[step] << std::endl;
        // servoing(gladiator,  curr_route[step]);


        /////////////////////////////////////////////////
        /////////////////// Servoing ////////////////////
        /////////////////////////////////////////////////
        // Get current Robot Data
        RobotData data = gladiator->robot->getData();
        Position position = data.position; // Location of the robot
        switch(etat){
            // Init with the current position/speed data of the robot
            case INIT:
            {
                speedG = (float)data.vl; // Maybe put the actual wheel speed here
                speedD = (float)data.vr;

                speedG = 0.0; // Maybe put the actual wheel speed here
                speedD = 0.0;

                pos0.x = position.x;
                pos0.y = position.y;
                pos0.a = position.a;
                // Condition pour commencer le match, ou pour passer au premier point
                etat = DESIRED_POSITION;
                break;
            }
            // Here we depile the route of the robot
            case DESIRED_POSITION:
            {
                // TODO: manage linear movements instead of case by case
                MazeSquare targetSquare = gladiator->maze->getSquare(curr_route[step][0], curr_route[step][1]);
                startingSquare = targetSquare;
                posd.x = (targetSquare.i + 0.5) * squareSize;
                posd.y = (targetSquare.j + 0.5) * squareSize; 
                gladiator->log("Target Square : (%d, %d)", targetSquare.i, targetSquare.j); 
                gladiator->log("Target Square : (%f, %f)", posd.x, posd.y);      

                distd = sqrt((posd.x-pos0.x)*(posd.x-pos0.x) + (posd.y-pos0.y)*(posd.y-pos0.y));
                thd = atan2((posd.y-pos0.y),(posd.x-pos0.x));// - pos0.a;

                // Going backward if it takes less time
                if(thd-pos0.a>M_PI/2){
                    backward = true;
                    thd -= M_PI;
                }else if(thd-pos0.a<-M_PI/2){
                    backward = true;
                    thd += M_PI;
                }else{
                    backward=false;
                }

                step += 1;

                etat = ANGLE_AND_DISTANCE;
                break;
            }
            // We manage first angle servoing
            case ANGLE_AND_DISTANCE:
            {
                erra = thd-position.a;
                if(erra>M_PI){
                    erra-=2*M_PI;
                } else if(erra<-M_PI){
                    erra+=2*M_PI;
                }
                errd = distd - sqrt((pos0.x-position.x)*(pos0.x-position.x) + (pos0.y-position.y)*(pos0.y-position.y));

                // We reached our target !
                if(abs(errd)<0.005){
                    etat = INIT;
                    if (step == maxStep)
                    {
                        hasObjective = false;
                    }
                
                } else {
                    // Angle servoing mode
                    if(abs(erra)>0.05)
                    {
                        errd = 0.01;
                    // Translation servoing mode
                    } else {
                        erra = 0.0;
                    }
                }
                break;
            }
            // If we want to face the ennemy *ç*
            // TODO later
            case ANGLE_FINAL:
            {
                // TODO: compute angle to reach at the end of the movement posd.a
                posd.a = 0.0; 
                erra = posd.a-position.a;
                errd = 0.0;

                /*if(abs(erra)<0.001){
                    etat = INIT;
                    delay(1000);
                }*/
                break;
            }
            default:
                break;

        }

        // Proportionnal position control
        float kpa = 0.4;
        float kpd = 40.0;
        speedG = -erra*kpa + RobotRadius*errd*kpd;
        speedD = +erra*kpa + RobotRadius*errd*kpd;
        /*if(backward){
            speedG = -speedG;
            speedD = -speedD;
        }*/
        if(!backward){
            speedG = -erra*kpa + RobotRadius*errd*kpd;
            speedD = +erra*kpa + RobotRadius*errd*kpd;
        }else{
            speedG = -erra*kpa - RobotRadius*errd*kpd;
            speedD = +erra*kpa - RobotRadius*errd*kpd;
        }

        // Saturation
        float speedMax = 0.4;
        if(speedG>speedMax){
            speedG = speedMax;
        }else if(speedG<-speedMax){
            speedG =-speedMax;
        }

        if(speedD>speedMax){
            speedD = speedMax;
        }else if(speedD<-speedMax){
            speedD =-speedMax;
        }

        gladiator->control->setWheelSpeed(WheelAxis::LEFT, speedG);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, speedD);

        //gladiator->log("Step:%d",step);
        /*
        gladiator->log("Position:%f,%f,%f",position.x,position.y,position.a);
        gladiator->log("Absolute desired position:%f,%f,%f",posd.x, posd.y,posd.a);
        gladiator->log("Desired position:%f,%f",distd,thd);
        gladiator->log("Erreur:%f,%f",erra,errd);
        gladiator->log("Etat:%i",etat);
        */
       

        // Display current time
        endTime = std::chrono::system_clock::now();
        //std::cout << "Time in seconds : " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << std::endl;

        delay(10);
    }
    // La consigne en vitesse est forcée à 0 lorsque aucun match n'a débuté.
}
