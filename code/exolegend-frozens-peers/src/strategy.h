#include <vector>

//using Path = std::vector<MazeSquare>;
using Path = std::vector<std::vector<uint8_t>>;


/**
 * @brief      Compute route to the closest Diamond in the arena using BFS
 *
 * @param      gladiator     link to the gladiator
 * 
 * @return     list of list of successive squares to reach.
 */
std::vector<Path> BFSPathToClosestDiamond(Gladiator* gladiator, MazeSquare source, int depth, int time, unsigned char ourId, unsigned char allyId, std::vector<unsigned char> ennemyIds);


/**
 * @brief      Display an input rotue in the debug window
 * 
 * @param      gladiator     link to the gladiator
 * 
 * @param      gpathsToDisplay     path to display
 */
void displayPaths(Gladiator* gladiator, std::vector<Path> pathsToDisplay);


/**
 * @brief      Manage robot servoing for reaching a location
 * 
 * @param      gladiator     link to the gladiator
 * 
 * @param      targetSquare     link to the target square
 */
void servoing(Gladiator* gladiator,  MazeSquare targetSquare);