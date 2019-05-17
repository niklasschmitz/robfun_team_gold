#ifndef SRC_MAZE_H
#define SRC_MAZE_H

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

        const T_VECTOR2D rotate(double theta) const {
            double newX = x * cos(theta) - y * sin(theta);
            double newY = x * sin(theta) + y * cos(theta);
            return T_VECTOR2D(newX, newY);
        }

        // returns a new cell which is this cell rotated i * 90 degrees counter-clockwise
        const Cell rotate(uint i) const {
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

    };

}

#endif //SRC_MAZE_H
