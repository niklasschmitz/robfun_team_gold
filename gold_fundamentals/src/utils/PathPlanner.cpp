#include "PathPlanner.h"
#include <queue>


Maze* PathPlanner::breadthFirstSearch(const Maze& maze, const Cell& start) {
    Maze* treeOfShortestPaths = new Maze(maze); //TODO

    std::queue<Cell> queue;
    Cell v = start; //TODO copy constructor? i.e. create new?
    queue.push(v);
    while (queue.size() > 0) {
        v = queue.front();
        queue.pop();
        for (Cell neighbor : treeOfShortestPaths->getNeighbors(v)) {
            if (neighbor.visited) {
                continue;
            }
            neighbor.visited = true;
            neighbor.predecessor = &v;
            queue.push(neighbor);
        }
    }

    return treeOfShortestPaths;
}