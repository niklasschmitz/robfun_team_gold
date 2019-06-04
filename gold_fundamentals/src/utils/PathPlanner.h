#ifndef SRC_PATHPLANNER_H
#define SRC_PATHPLANNER_H

#include "Maze.h"

using namespace maze;

class PathPlanner {

    /**
     *
     * @param maze
     * @param goal
     * @return tree with all cells of the maze, having predecessors set.
     *         essentially represents all shortest paths from the goal to any other node
     */
    std::vector<Cell> PathPlanner::breadthFirstSearch(const Maze& maze, Cell start);
};


#endif //SRC_PATHPLANNER_H
