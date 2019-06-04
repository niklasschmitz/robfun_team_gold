#ifndef SRC_PATHPLANNER_H
#define SRC_PATHPLANNER_H

#include "Maze.h"

using namespace maze;

class PathPlanner {
public:
    /**
     *
     * @param maze
     * @param goal
     * @return tree with all cells of the maze, having predecessors set.
     *         essentially represents all shortest paths from the goal to any other node
     */
    Maze* breadthFirstSearch(const Maze& maze, const Cell& start);
};


#endif //SRC_PATHPLANNER_H
