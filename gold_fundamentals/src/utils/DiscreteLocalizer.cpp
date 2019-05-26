#include "DiscreteLocalizer.h"
#include "GridPerceptor.h"

DiscreteLocalizer::DiscreteLocalizer() {
    int n_cols = maze::Maze::MAZE_WIDTH;
    int n_rows = maze::Maze::MAZE_HEIGHT;

    // populate candidates with all possible aligned configurations
    // this can be thought of as a uniform prio
    for (int row = 0; row < n_rows; ++row) {
        for (int col = 0; col < n_cols; ++col) {
            for (int theta = 0; theta < 4; ++theta) {
                double x = (row + 0.5) * MAZE_SIDE_LENGTH;
                double y = (col + 0.5) * MAZE_SIDE_LENGTH;
                this->candidates.push_back(RobotConfiguration(x, y, (double) theta * M_PI_2));
            }
        }
    }
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
