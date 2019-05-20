#include "OccupancyGrid.h"
#include "GridPerceptor.h"
#include <sstream>

OccupancyGrid::OccupancyGrid() {}

OccupancyGrid::~OccupancyGrid() {
    delete[] grid_data;
}

void OccupancyGrid::printGrid() {
    ROS_INFO("PRINTING GRID");
    for( int row=0; row<height; row++) {
        std::ostringstream row_string;
        //row_string << row+1;
        for( int col=0; col<width; col++) {
            uint8_t val = grid_data[row * width + col];
            if(val == 0) {
                row_string << '-';
            } else {
                row_string << 'X';
            }
        }

        ROS_INFO("%s", row_string.str().c_str());
    }
}

// map = map from the node [[[T,L,R], ... ]]], inverse_resolution = pixels / meter
void OccupancyGrid::convertMsgGridToOccupancyGrid(const gold_fundamentals::Grid::ConstPtr &msg_grid, int inverse_res) {
    // find out max horizontal, vertical spread of the map (x=max_nr_of_cols, y=max_nr_of_rows)
    T_VECTOR2D msg_grid_dims = OccupancyGrid::getMsgGridDimensions(msg_grid);

    max_cells_x = static_cast<int>(msg_grid_dims.x);
    max_cells_y = static_cast<int>(msg_grid_dims.y);
    inverse_resolution = inverse_res;
    width = static_cast<int>(msg_grid_dims.x * MAZE_SIDE_LENGTH * inverse_resolution); // *80 as one cell is 80cm
    height = static_cast<int>(msg_grid_dims.y * MAZE_SIDE_LENGTH * inverse_resolution);

    // delete any memory allocated before
    delete[] grid_data;
    // create new data object
    grid_data = new uint8_t[width*height];

    setAllCellBorders(msg_grid);

}

void OccupancyGrid::setSingleGridPixel(int cell_x, int cell_y, int box_x, int box_y, uint8_t value) {
    int sideLengthInPixels = getBoxSideLengthInPixels();
    // pixels in one full row * (rows to go down because of cell offset + box_y offset) + (cols to go left because cell offset + box_x offset)
    // maybe sidelength-1 ?
    int index = max_cells_x * sideLengthInPixels * (cell_y * sideLengthInPixels + box_y) + sideLengthInPixels * cell_x + box_x;
    grid_data[index] = value;
}

int OccupancyGrid::getBoxSideLengthInPixels() {
    // maze_side_length [m], inverse_res [pixels/m]
    return MAZE_SIDE_LENGTH * inverse_resolution;
}

// gets max nr of rows & cols of the map coming from the node
T_VECTOR2D OccupancyGrid::getMsgGridDimensions(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    // grid -> rows -> cells -> walls
    T_VECTOR2D dims;
    // max nr of rows
    dims.y = msg_grid->rows.size();

    int max_nr_of_cols = 0;
    for(int row=0; row<msg_grid->rows.size(); row++) {
        int nr_of_cols = msg_grid->rows[row].cells.size();
        if(nr_of_cols > max_nr_of_cols) {
            max_nr_of_cols = nr_of_cols;
        }
    }

    dims.x = max_nr_of_cols;
    return dims;
}

OccupancyGrid::WallData OccupancyGrid::getWallData(const gold_fundamentals::Grid::ConstPtr &msg_grid, int col, int row) {
    // find out how many walls there are
    int nr_of_walls = msg_grid->rows[row].cells[col].walls.size();
    WallData wallData;
    wallData.Bot = false;
    wallData.Top = false;
    wallData.Right = false;
    wallData.Left = false;

    // find out which walls are set
    for(int wall_idx=0; wall_idx<nr_of_walls; wall_idx++) {
        switch (msg_grid->rows[row].cells[col].walls[wall_idx]) {
            case 0:
                wallData.Right = true;
                break;
            case 1:
                wallData.Top = true;
                break;
            case 2:
                wallData.Left = true;
                break;
            case 3:
                wallData.Bot = true;
                break;

            default:
                // shouldnt get here
                break;
        }
    }

    return wallData;
}

void OccupancyGrid::setAllCellBorders(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    for( int row=0; row<max_cells_y; row++ ) {
        for( int col=0; col<max_cells_x; col++ ) {
//            msg_grid->rows[row].cells[col].walls[wall_idx]
            WallData wallData = getWallData(msg_grid, col, row);
            setSingleCellBorders(col, row, wallData);
        }
    }
}

// sets a box directly in the global occupancy grid
void OccupancyGrid::setSingleCellBorders(int cell_x, int cell_y, WallData wallData) {
    // iterate over all pixels in the box and set to zero (reset)
    int boxSideLength = getBoxSideLengthInPixels();

    for( int row=0; row<boxSideLength; row++ ) {
        for( int col=0; col<boxSideLength; col++ ) {
            setSingleGridPixel(cell_x, cell_y, col, row, 0);
        }
    }

    // now set the borders
    if(wallData.Top) {
        for(int col=0; col<boxSideLength; col++) {
            setSingleGridPixel(cell_x, cell_y, col, 0, 100);
        }
    }

    if(wallData.Bot) {
        for(int col=0; col<boxSideLength; col++) {
            setSingleGridPixel(cell_x, cell_y, col, boxSideLength-1, 100);
        }
    }

    if(wallData.Left) {
        for(int row=0; row<boxSideLength; row++) {
            setSingleGridPixel(cell_x, cell_y, 0, row, 100);
        }
    }

    if(wallData.Right) {
        for(int row=0; row<boxSideLength; row++) {
            setSingleGridPixel(cell_x, cell_y, boxSideLength-1, row, 100);
        }
    }
}
