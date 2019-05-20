#include "OccupancyGrid.h"
#include "GridPerceptor.h"
#include <sstream>

OccupancyGrid::OccupancyGrid() {}

OccupancyGrid::~OccupancyGrid() {
    delete[] grid_data;
}

void OccupancyGrid::printGrid() {
    for( int row=0; row<height; row++) {
        std::ostringstream row_string;
        for( int col=0; col<width; col++) {
            uint8_t val = grid_data[row * width + col];
            row_string << val;
        }

        ROS_INFO("%s", row_string.str().c_str());
    }
}

// map = map from the node [[[T,L,R], ... ]]], inverse_resolution = pixels / meter
void OccupancyGrid::convertMsgGridToOccupancyGrid(const gold_fundamentals::Grid::ConstPtr &msg_grid, int inverse_resolution) {
    // find out max horizontal, vertical spread of the map (x=max_nr_of_cols, y=max_nr_of_rows)
    T_VECTOR2D msg_grid_dims = OccupancyGrid::getMsgGridDimensions(msg_grid);

    max_cells_x = static_cast<int>(msg_grid_dims.x);
    max_cells_y = static_cast<int>(msg_grid_dims.y);
    //resolution = 1 / inverse_resolution;
    width = static_cast<int>(msg_grid_dims.x * MAZE_SIDE_LENGTH * inverse_resolution); // *80 as one cell is 80cm
    height = static_cast<int>(msg_grid_dims.y * MAZE_SIDE_LENGTH * inverse_resolution);

    // delete any memory allocated before
    delete[] grid_data;
    // create new data object
    grid_data = new uint8_t[width*height];

    std::vector<OccupancyGrid::Box*> boxes = createBoxesFromMsgGrid(msg_grid, inverse_resolution);
    // iterate over boxes and set data accordingly
    for( int box_idx=0; box_idx<boxes.size(); box_idx++) {
        for( int row=0; row<boxes[box_idx]->sideLengthInPixels; row++) {
            for( int col=0; col<boxes[box_idx]->sideLengthInPixels; col++) {
                uint8_t box_pixel_value = boxes[box_idx]->getSingleBoxPixel(col, row);
                setSingleGridPixel(boxes[box_idx]->cell_pos_x, boxes[box_idx]->cell_pos_y, col, row, box_pixel_value, inverse_resolution);
            }
        }
    }

    //delete unneeded boxes again
    for( int i=0; i<boxes.size(); i++) {
        delete boxes[i];
    }
}

void OccupancyGrid::setSingleGridPixel(int cell_x, int cell_y, int box_x, int box_y, uint8_t value, int inverse_resolution) {
    int sideLengthInPixels = getBoxSideLengthInPixels(inverse_resolution);
    // pixels in one full row * (rows to go down because of cell offset + box_y offset) + (cols to go left because cell offset + box_x offset)
    int index = max_cells_x * sideLengthInPixels * (cell_y * sideLengthInPixels + box_y) + sideLengthInPixels * cell_x + box_x;
    grid_data[index] = value;
}

int OccupancyGrid::getBoxSideLengthInPixels(int inverse_resolution) {
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

std::vector<OccupancyGrid::Box*>
OccupancyGrid::createBoxesFromMsgGrid(const gold_fundamentals::Grid::ConstPtr &msg_grid, int inverse_resolution) {
    std::vector<OccupancyGrid::Box*> boxes;
    for(int row=0; row<max_cells_y; row++) {
        for(int col=0; col<max_cells_x; col ++) {
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

            // create the box
            OccupancyGrid::Box* box = new Box(col, row, inverse_resolution);
            box->setBorders(wallData);

            boxes.push_back(box);
        }
    }

    return boxes;
}


OccupancyGrid::Box::Box(uint8_t cell_pos_x, uint8_t cell_pos_y, int inverse_resolution)
    : cell_pos_x(cell_pos_x), cell_pos_y(cell_pos_y){
    sideLengthInPixels = OccupancyGrid::getBoxSideLengthInPixels(inverse_resolution);
    box_data = new uint8_t[sideLengthInPixels * sideLengthInPixels]; // () to init to zero
    std::fill(box_data, box_data+sideLengthInPixels*sideLengthInPixels, 0);
}

OccupancyGrid::Box::~Box() {
    delete[] box_data;
}

void OccupancyGrid::Box::setBorders(OccupancyGrid::WallData mwp) {
    // box data is initialized to zero
    if (mwp.Bot == true) {
       for(int col=0; col<sideLengthInPixels; col++) {
           setSingleBoxPixel( sideLengthInPixels, col, true);
       }
    }

    if (mwp.Top == true) {
        for(int col=0; col<sideLengthInPixels; col++) {
            setSingleBoxPixel( 0, col, true);
        }
    }

    if (mwp.Left == true) {
        for(int row=0; row<sideLengthInPixels; row++) {
            setSingleBoxPixel( row, 0, true);
        }
    }

    if (mwp.Right == true) {
        for(int row=0; row<sideLengthInPixels; row++) {
            setSingleBoxPixel( row, sideLengthInPixels, true);
        }
    }
}

void OccupancyGrid::Box::setSingleBoxPixel( int x, int y, bool occupied) {
    if(occupied == true) {
        box_data[sideLengthInPixels*y + x] = 100;
    } else {
        // not occupied
        box_data[sideLengthInPixels*y + x] = 0;
    }
}

uint8_t OccupancyGrid::Box::getSingleBoxPixel( int x, int y) {
    return box_data[sideLengthInPixels*y + x];
}
