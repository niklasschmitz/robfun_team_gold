#include "DiscreteLocalizer.h"

void DiscreteLocalizer::DiscreteLocalizer() {

}

void DiscreteLocalizer::estimateConfiguration(RobotConfiguration action, maze::Cell observation) {
    std::vector<RobotConfiguration> new_candidates;

    for (int i = 0; i < this->candidates.size(); ++i) {

        // project old candidate into future position according to discrete motion model
        RobotConfiguration candidate = candidates[i] + action;

        // get expected cell perception
        Cell expected_cell = this->map.getCell(candidate.position);

        // check if it still matches observation //TODO: make sure rotation is correct
        if (expected_cell == observation.rotate((uint) (2 * candidate.theta / M_PI))) {
            new_candidates.push_back(expected_cell);
        }

    }

    delete(candidates);
    this->candidates = new_candidates;
}
