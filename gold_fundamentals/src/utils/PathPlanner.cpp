#include "PathPlanner.h"
#include <queue>


void PathPlanner::populateBreadthFirstSearch(const Maze &maze, Cell* goal) {
    delete(treeOfShortestPaths);
    treeOfShortestPaths = new Maze(maze);

    std::queue<Cell*> queue;
    Cell* v = goal; //TODO copy constructor? i.e. create new?
    queue.push(v);
    while (queue.size() > 0) {
        v = queue.front();
        queue.pop();
        for (Cell* neighbor : treeOfShortestPaths->getNeighbors(v)) {
            if (!neighbor->visited) {
                neighbor->visited = true;
                neighbor->predecessor = v;
                queue.push(neighbor);
            }
        }
    }

}

std::vector<int> PathPlanner::extractPlan(Cell* start, Cell* goal) {
    //TODO make sure BFS from goal node has been executed before to populate predecessor pointers
    if (!start || !goal) {
        return std::vector<int>();
    }

    std::vector<int> plan;
    Cell* v = start;
    while (v != goal && v->predecessor) {
        if (v->predecessor->col > v->col) { // right
            plan.emplace_back(0);
        } else if (v->predecessor->row < v->row) { // up
            plan.emplace_back(1);
        } else if (v->predecessor->col < v->col) { // left
            plan.emplace_back(2);
        } else if (v->predecessor->row > v->row) { // down
            plan.emplace_back(3);
        }
        v = v->predecessor;
    }

    return plan;
}
