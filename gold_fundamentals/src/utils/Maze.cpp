#include "Maze.h"

namespace maze {

    Maze::Maze(int n_rows, int n_cols) {
        this->n_rows = n_rows;
        this->n_cols = n_cols;
        this->map = new Cell[n_rows * n_cols];
    }

    const Cell Maze::getCell(int row, int col) {
        return map[row*n_cols + col];
    }

    const Cell Maze::getCell(T_VECTOR2D position) {
        int x = (int) position.x;
        int y = (int) position.y;
        return this->getCell(x, y);
    }

    void Maze::setCell(int row, int col, Cell cell) {
        map[row*n_cols + col] = cell;
    }


} // namespace
