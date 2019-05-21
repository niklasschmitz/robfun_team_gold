#include "Maze.h"

using namespace maze;

const Cell Maze::getCell(T_VECTOR2D position) {
    int x = (int) position.x;
    int y = (int) position.y;
    return this->map[x][y];
}
