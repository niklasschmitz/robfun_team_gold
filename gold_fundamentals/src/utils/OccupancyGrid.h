#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry.h"

#include <cstdlib>
#include <ctime>

//class Odometry {
//
//};

class OccupancyGrid {
public:

    OccupancyGrid();

    struct MapWallData {
        bool Top;
        bool Right;
        bool Bot;
        bool Left;
    };

    class Box {
    public:

        uint8_t cell_pos_x;
        uint8_t cell_pos_y;

        Box(int inverse_resolution);
        void setBorders(MapWallData mwp);

    private:
        uint8_t* data;

    };

    void convertMapToOccupancyGrid(std::vector<int> map, int inverse_resolution);

    // sets occupancy (0 = not occupied, 1 = occupied)
    void setMazeBoxOccupancy(int row, int col, MapWallData);

    // the maze consists of boxes. x and y are the horizontal and vertical.
    // row and col are the cell params in the box. box cell side length is the side lenght of a box measured in cells.

    //nt computeGridCellIndex(int box_x, int box_y, int row, int col, int box_cell_side_length);
    // gets width and height of the map coming from the node
    T_VECTOR2D getMapSizeData();

private:
    // cells are those in the actual labyrinth
    // boxes are discretized versions of cells, they have pixels

    int max_cells_x; // how many cells there are in the x direction
    int max_cells_y;
    int width; // in pixel
    int height; // in pixel
    double resolution; // meter / pixel
    uint8_t* data;

};

#endif
