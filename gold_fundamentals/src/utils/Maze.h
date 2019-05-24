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
        static const int MAZE_HEIGHT = 3;
        static const int MAZE_WIDTH = 5;
        Cell map[MAZE_HEIGHT][MAZE_WIDTH];

        const Cell getCell(T_VECTOR2D position);
    };

}

#endif //SRC_MAZE_H
