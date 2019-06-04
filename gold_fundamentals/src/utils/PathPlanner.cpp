#include "PathPlanner.h"
#include <queue>


std::vector<Cell> PathPlanner::breadthFirstSearch(const Maze& maze, Cell start) {
    std::vector<Cell> treeOfShortestPaths; //TODO

    std::queue<Cell> queue;
    Cell v = start; //TODO copy constructor? i.e. create new?
    queue.push(v);
    while (queue.size() > 0) {
        v = queue.pop(); //TODO =
        for (Cell neighbor : maze.getNeighbors(v)) {
            if (neighbor.visited) {
                continue;
            }
            neighbor.visited = true;
            neighbor.predecessor = v; // TODO =
            queue.push(neighbor);
        }
    }

    return treeOfShortestPaths;
}