#include "OccupancyGrid.h"


OccupancyGrid::OccupancyGrid() {}

// map = map from the node [[[T,L,R], ... ]]], inverse_resolution = cells / meter
void OccupancyGrid::convertMapToOccupancyGrid(std::vector<int> map, int inverse_resolution) {
//    T_VECTOR2D map_data = getMapData();
//    resolution = 1 / inverse_resolution;
//    width = map_data.x * MAZE_SIDE_LENGTH * inverse_resolution; // *80 as one cell is 80cm
//    height = map_data.y * MAZE_SIDE_LENGTH * inverse_resolution;
//
//    // delete data[] ?
//    data = new uint8_t[width*height];
//
//    for (int col=0; col<map_data.y ; ++col) {
//        for (int row=0< row<map_data.x; ++row) {
//            // iterate map row wise, wall_index element T,L,R,B
//            MapWallData mwd = getMapWallData(map[col][row]);
//            setMazeBoxOccupancy(row, col, mwd);
//        }
//    }
}

// sets occupancy (0 = not occupied, 1 = occupied)
void OccupancyGrid::setMazeBoxOccupancy(int row, int col, MapWallData) {
    // how many cells are there in the box
//    int cell_side_length_of_box = floor(MAZE_SIDE_LENGTH / resolution);
//
//    for(int side_length_col=0; side_length_col<cell_side_length_of_box; side_length_col++) {
//        for (int side_length_row=0; side_length_row<cell_side_length_of_box; side_length_row++) {
//            data[]
//        }
//    }
}

// the maze consists of boxes. x and y are the horizontal and vertical.
// row and col are the cell params in the box. box cell side length is the side lenght of a box measured in cells.
//int OccupancyGrid::computeGridCellIndex(int box_x, int box_y, int row, int col, int box_cell_side_length) {
//
//    return box_x * box_cell_side_length;
//}

// gets width and height of the map coming from the node
T_VECTOR2D OccupancyGrid::getMapSizeData() {

}

OccupancyGrid::Box::Box(int inverse_resolution) {

}
