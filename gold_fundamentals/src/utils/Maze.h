#ifndef SRC_MAZE_H
#define SRC_MAZE_H

#include "geometry.h"

namespace maze {

    /**
     * The CellWallData type stores 4 booleans indicating which walls are present.
     * Note: It does not store its location within the global maze. For
     * that, see class 'Maze'
     */
    struct CellWallData {
        bool right;
        bool top;
        bool left;
        bool bottom;

        // constructor
        CellWallData(bool right = false, bool top = false, bool left = false, bool bottom = false)
                : right(right), top(top), left(left), bottom(bottom) {}

        // equality
        bool operator==(const CellWallData &other) {
            return (right == other.right && top == other.top
                    && left == other.left && bottom == other.bottom);
        }


        // returns a new cell which is this cell rotated i * 90 degrees counter-clockwise
        const CellWallData rotate(int i) const {
            i = i % 4;

            if (i == 0) {
                return CellWallData(right, top, left, bottom);
            } else if (i == 1) {
                return CellWallData(bottom, right, top, left);
            } else if (i == 2) {
                return CellWallData(left, bottom, right, top);
            } else {
                return CellWallData(top, left, bottom, right);
            }
        }

        void set(int direction, bool hasWall) {
            direction %= 4;

            if (direction == 0) {
                right = hasWall;
            } else if (direction == 1) {
                top = hasWall;
            } else if (direction == 2) {
                left = hasWall;
            } else {
                bottom = hasWall;
            }
        }
    };

    struct Cell {
        int row;
        int col;
        CellWallData walls;

        // for breadth-first search
        bool visited;
        Cell* predecessor;

        Cell(int row = 0, int col = 0, bool visited = false, Cell* predecessor = nullptr)
                    : row(row), col(col), visited(visited), predecessor(predecessor) {}

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

        std::vector<CellWallData> map;

        Maze(const int nCols = 0, const int nRows = 0, const double cellSideLength = 0.8);

        ~Maze();

        const CellWallData getCell(int row, int col);

        // returns the discrete Cell to which the
        // (continuous) position is mapped
        const CellWallData getCell(T_VECTOR2D position);

        void setCell(int row, int col, CellWallData cell);

        std::vector<Coordinate> getNeighbors(Coordinate coordinate);

        std::vector<Coordinate> breadthFirstSearch(Coordinate goal);
    };

} // namespace

#endif //SRC_MAZE_H
