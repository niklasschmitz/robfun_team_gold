#include "PathPlanner.h"
#include <queue>


Maze* PathPlanner::breadthFirstSearch(const Maze& maze, Cell* start) {
    Maze* treeOfShortestPaths = new Maze(maze); //TODO

    std::queue<Cell*> queue;
    Cell* v = start; //TODO copy constructor? i.e. create new?
    queue.push(v);
    while (queue.size() > 0) {
        v = queue.front();
        queue.pop();
        for (Cell* neighbor : treeOfShortestPaths->getNeighbors(v)) {
            // TODO this is buggy because we dont change neighbors in-place! getNeighbors returns copy
            if (!neighbor->visited) {
                neighbor->visited = true;
                neighbor->predecessor = v;
                queue.push(neighbor);
            }
        }
    }

    return treeOfShortestPaths;
}