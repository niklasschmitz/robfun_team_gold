#include "Maze.h"

namespace maze {

    Maze::Maze(const int nCols, const int nRows, const double cellSideLength)
            : n_cols(nCols), n_rows(nRows), CELL_SIDE_LENGTH(cellSideLength) {}

    const Cell Maze::getCell(int row, int col) {
        return map[row * n_cols + col];
    }

    const Cell Maze::getCell(T_VECTOR2D position) {
        int x = (int) position.x;
        int y = (int) position.y;
        return this->getCell(x, y);
    }

    void Maze::setCell(int row, int col, Cell cell) {
        map[row * n_cols + col] = cell;
    }

} // namespace
