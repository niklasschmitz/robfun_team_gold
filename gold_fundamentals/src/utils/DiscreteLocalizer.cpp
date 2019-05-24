#include "DiscreteLocalizer.h"

DiscreteLocalizer::DiscreteLocalizer() {
    int n_cols = maze::Maze::MAZE_WIDTH;
    int n_rows = maze::Maze::MAZE_HEIGHT;

    // populate candidates with all possible aligned configurations
    // this can be thought of as a uniform prio
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            for (int theta = 0; theta < 4; ++theta) {
                this->candidates.push_back(RobotConfiguration((double) i + 0.5, (double) j + 0.5, (double) theta * M_PI_2));
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
        if (expected_cell == observation.rotate((uint) (2 * candidate.theta / M_PI))) {
            new_candidates.push_back(candidate);
        }

    }
    //TODO: make sure old candidates array memory is free'd
    this->candidates = new_candidates;
}
