#ifndef SRC_PATHPLANNER_H
#define SRC_PATHPLANNER_H

#include "Maze.h"

using namespace maze;

class PathPlanner {
private:
    Cell* goal;
    Maze* treeOfShortestPaths;

public:
    /**
     * @param maze
     * @param goal
     * builds tree with all cells of the maze, having predecessors set.
     * essentially represents all shortest paths from the goal to any other node
     */
    void populateBreadthFirstSearch(const Maze& maze, Cell* goal);

    std::vector<int> extractPlan(Cell* start, Cell* goal);
};


#endif //SRC_PATHPLANNER_H
