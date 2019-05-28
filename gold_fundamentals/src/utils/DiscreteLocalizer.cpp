#include "DiscreteLocalizer.h"

DiscreteLocalizer::DiscreteLocalizer() {
    int n_cols = maze::Maze::N_COLS;
    int n_rows = maze::Maze::N_ROWS;

    // populate candidates with all possible aligned configurations
    // this can be thought of as a uniform prio
    for (int row = 0; row < n_rows; ++row) {
        for (int col = 0; col < n_cols; ++col) {
            for (int theta = 0; theta < 4; ++theta) {
                double x = (row + 0.5) * maze::Maze::CELL_SIDE_LENGTH;
                double y = (col + 0.5) * maze::Maze::CELL_SIDE_LENGTH;
                this->candidates.push_back(RobotConfiguration(x, y, (double) theta * M_PI_2));
            }
        }
    }
}

DiscreteLocalizer::~DiscreteLocalizer(){}

// map = map from the node [[[T,L,R], ... ]]]
void OccupancyGrid::convertMsgGridToMap(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    // find out max horizontal, vertical spread of the map (x=max_nr_of_cols, y=max_nr_of_rows)

    int rows = static_cast<int>(msg_grid_dims.x);
    int cols = static_cast<int>(msg_grid_dims.y);
    inverse_resolution = inverse_res;
    width = static_cast<int>(msg_grid_dims.x * MAZE_SIDE_LENGTH * inverse_resolution); // *80 as one cell is 80cm
    height = static_cast<int>(msg_grid_dims.y * MAZE_SIDE_LENGTH * inverse_resolution);

    // delete any memory allocated before
    delete[] grid_data;
    // create new data object
    grid_data = new uint8_t[width*height];

    setAllCellBorders(msg_grid);

}

void DiscreteLocalizer::estimateConfiguration(RobotConfiguration action, maze::Cell observation) {
    std::vector<RobotConfiguration> new_candidates;

    for (int i = 0; i < this->candidates.size(); ++i) {

        // project old candidate into future position according to discrete motion model
        RobotConfiguration candidate = candidates[i] + action;

        // get expected cell perception
        maze::Cell expected_cell = this->map.getCell(candidate.position);

        // check if it still matches observation //TODO: make sure rotation is correct
        if (expected_cell == observation.rotate((int) (2 * candidate.theta / M_PI))) {
            new_candidates.push_back(candidate);
        }

    }
    //TODO: make sure old candidates array memory is free'd
    this->candidates = new_candidates;
}
