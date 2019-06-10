#include "Maze.h"

namespace maze {

    Maze::Maze(const int nCols, const int nRows, const double cellSideLength)
            : n_cols(nCols), n_rows(nRows), CELL_SIDE_LENGTH(cellSideLength) {}

    Cell *Maze::getCell(int row, int col) {
        return map[row * n_cols + col];
    }

    Cell *Maze::getCell(T_VECTOR2D position) {
        int x = (int) position.x;
        int y = (int) position.y;
        return this->getCell(x, y);
    }

    void Maze::setCell(int row, int col, CellWallData walls) {
        Cell *cell = new Cell();
        cell->row = row;
        cell->col = col;
        cell->walls = walls;
        map[row * n_cols + col] = cell;
    }

    std::vector<Cell*> Maze::getNeighbors(Cell *cell) {
        std::vector<Cell*> neighbors;

        if (!cell->walls.right && isValidIndex(cell->row, cell->col + 1)) {
            neighbors.push_back(this->getCell(cell->row, cell->col + 1));
        }
        if (!cell->walls.top && isValidIndex(cell->row - 1, cell->col)) {
            neighbors.push_back(this->getCell(cell->row - 1, cell->col));
        }
        if (!cell->walls.left && isValidIndex(cell->row, cell->col - 1)) {
            neighbors.push_back(this->getCell(cell->row, cell->col - 1));
        }
        if (!cell->walls.bottom && isValidIndex(cell->row + 1, cell->col)) {
            neighbors.push_back(this->getCell(cell->row + 1, cell->col));
        }

        return neighbors;
    }

    bool Maze::isValidIndex(int row, int col) const {
        return (row >= 0 && row < this->n_rows && col >= 0 && col < this->n_cols);
    }

    Maze::~Maze() {

    }

} // namespace
