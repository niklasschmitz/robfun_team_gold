#ifndef SRC_MAZE_H
#define SRC_MAZE_H

#include "geometry.h"

namespace maze {



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

    class Maze {
    public:
        // number of rows and columns
        static const int N_ROWS = 3;
        static const int N_COLS = 5;

        // side length of a single cell in cm
        static const double CELL_SIDE_LENGTH = 0.8;

        Cell map[N_ROWS][N_COLS];

        const Cell getCell(T_VECTOR2D position);
    };

}

#endif //SRC_MAZE_H
