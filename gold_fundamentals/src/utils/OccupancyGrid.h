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
        bool Bot;
        bool Left;
    };

    class Box {
    public:

        uint8_t cell_pos_x;
        uint8_t cell_pos_y;
        int sideLengthInPixels;

        Box(uint8_t cell_pos_x, uint8_t cell_pos_y, int inverse_resolution);
        ~Box();
        void setBorders(WallData mwp);

        void setSingleBoxPixel(int x, int y, bool occupied);
        uint8_t getSingleBoxPixel(int x, int y);
    private:
        uint8_t* box_data; // occupation information, 0-100% (likeliness to be occupied)

    };

    void convertMsgGridToOccupancyGrid(const gold_fundamentals::Grid::ConstPtr &grid, int inverse_resolution);

    void printGrid();

    // the maze consists of cells. x and y represent horizontal and vertical.
    // row and col are the box params in the cell. box pixel side length is the side lenght of a box measured in pixels.

private:
    // cells are those in the actual labyrinth
    // boxes are discretized versions of cells, they have pixels

    int max_cells_x; // how many cells there are in the x direction
    int max_cells_y;
    int width; // in pixel
    int height; // in pixel
    //double resolution; // meter / pixel
    uint8_t* grid_data; // occupation information, 0-100% (likeliness to be occupied)

    static T_VECTOR2D getMsgGridDimensions(const gold_fundamentals::Grid::ConstPtr &msg_grid);

    // box is a single cell expressed in pixels
    static int getBoxSideLengthInPixels(int inverse_resolution);

    std::vector<Box*> createBoxesFromMsgGrid(const gold_fundamentals::Grid::ConstPtr &msg_grid, int inverse_resolution);

    void setSingleGridPixel(int cell_x, int cell_y, int box_x, int box_y, uint8_t value, int inverse_resolution);
};

#endif
