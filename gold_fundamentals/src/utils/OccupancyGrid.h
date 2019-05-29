#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry.h"
#include "gold_fundamentals/Grid.h"

#include <cstdlib>
#include <ctime>

//class Odometry {
//
//};

class OccupancyGrid {
public:

    OccupancyGrid();
    ~OccupancyGrid();

    struct WallData {
        bool Top;
        bool Right;
        bool Bottom;
        bool Left;
    };

    void convertMsgGridToOccupancyGrid(const gold_fundamentals::Grid::ConstPtr &grid, int inverse_resolution);

    void printGrid();

    // the maze consists of cells. x and y represent horizontal and vertical.
    // cells are those in the actual labyrinth

    int max_cells_x; // how many cells there are in the x direction
    int max_cells_y;
    int width; // in pixel
    int height; // in pixel
    int inverse_resolution; // pixel / meter
    uint8_t* grid_data; // occupation information, 0-100% (likeliness to be occupied)

private:
    // boxes are discretized versions of cells, they have pixels

    static T_VECTOR2D getMsgGridDimensions(const gold_fundamentals::Grid::ConstPtr &msg_grid);

    // box is a single cell expressed in pixels
    int getBoxSideLengthInPixels();

    void setSingleGridPixel(int cell_x, int cell_y, int box_x, int box_y, uint8_t value);

    static WallData getWallData(const gold_fundamentals::Grid::ConstPtr &msg_grid, int cell_x, int cell_y);

    void setSingleCellBorders(int cell_x, int cell_y, WallData wallData);

    void setAllCellBorders(const gold_fundamentals::Grid::ConstPtr &msg_grid);
};

#endif
