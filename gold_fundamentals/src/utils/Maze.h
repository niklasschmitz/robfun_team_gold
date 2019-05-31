#ifndef SRC_MAZE_H
#define SRC_MAZE_H

#include "geometry.h"

namespace maze {

    /**
     * The Cell type stores 4 booleans indicating which walls are present.
     * Note: It does not store its location withing the global maze. For
     * that, see class 'Maze'
     */
    struct Cell {
        bool right;
        bool top;
        bool left;
        bool bottom;

        // constructor
        Cell(bool right = false, bool top = false, bool left = false, bool bottom = false)
                : right(right), top(top), left(left), bottom(bottom) {}

        // equality
        bool operator==(const Cell &other) {
            return (right == other.right && top == other.top
                    && left == other.left && bottom == other.bottom);
        }


        // returns a new cell which is this cell rotated i * 90 degrees counter-clockwise
        const Cell rotate(int i) const {
            i = i % 4;

            if (i == 0) {
                return Cell(right, top, left, bottom);
            } else if (i == 1) {
                return Cell(bottom, right, top, left);
            } else if (i == 2) {
                return Cell(left, bottom, right, top);
            } else {
                return Cell(top, left, bottom, right);
            }
        }
    };

    /**
     * A Maze object acts as an interface to a 2D array of Cells
     * which describes the topology of the maze.
     * Additionally it stores the CELL_SIDE_LENGTH.
     */
    class Maze {
    public:
        // number of rows and columns
        const int n_rows;
        const int n_cols;

        // side length of a single cell in m
        const double CELL_SIDE_LENGTH;

        std::vector<Cell> map;

        Maze(const int nCols = 0, const int nRows = 0, const double cellSideLength = 0.8);

        ~Maze();

        const Cell getCell(int row, int col);

        // returns the discrete Cell to which the
        // (continuous) position is mapped
        const Cell getCell(T_VECTOR2D position);

        void setCell(int row, int col, Cell cell);
    };

} // namespace

#endif //SRC_MAZE_H
