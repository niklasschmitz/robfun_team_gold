#include "DiscreteLocalizer.h"

DiscreteLocalizer::DiscreteLocalizer() {}

DiscreteLocalizer::~DiscreteLocalizer() {}

// TODO: Subscribe to mapPublisher
// map = map from the node [[[T,L,R], ... ]]]
void DiscreteLocalizer::convertMsgGridToMap(const gold_fundamentals::Grid::ConstPtr &msg_grid) {

    int n_rows = msg_grid->rows.size();
    int n_cols = n_rows ? msg_grid->rows[0].cells.size() : 0;

    // for every coordinate
    for (int row = 0; row < n_rows; ++row) {
        for (int col = 0; col < n_cols; ++col) {
            // set Cell booleans for each wall present

            int n_walls = msg_grid->rows[row].cells[col].walls.size();
            maze::Cell cell;

            // find out which walls are set
            for (int wall_idx = 0; wall_idx < n_walls; wall_idx++) {
                switch (msg_grid->rows[row].cells[col].walls[wall_idx]) {
                    case 0:
                        cell.right = true;
                        break;
                    case 1:
                        cell.top = true;
                        break;
                    case 2:
                        cell.left = true;
                        break;
                    case 3:
                        cell.bottom = true;
                        break;

                    default:
                        // shouldnt get here
                        break;
                }
            }

            // place cell on map
            maze->map.push_back(cell);
        }
    }

}

void DiscreteLocalizer::populateCandidates() {
    // populate candidates with all possible aligned configurations
    // this can be thought of as a uniform prior over states
    this->candidates.clear();
    for (int row = 0; row < maze->n_rows; ++row) {
        for (int col = 0; col < maze->n_cols; ++col) {
            for (int orientation = 0; orientation < 4; ++orientation) {
                gold_fundamentals::Pose pose;
                pose.row = row;
                pose.column = col;
                pose.orientation = orientation;
                this->candidates.push_back(pose);
            }
        }
    }
}

void DiscreteLocalizer::estimateConfiguration(gold_fundamentals::Pose action, maze::Cell observation) {
    std::vector<gold_fundamentals::Pose> new_candidates;

    for (int i = 0; i < this->candidates.size(); ++i) {

        // project old candidate into future position according to discrete motion model
        gold_fundamentals::Pose candidate = candidates[i];// + action; //TODO

        // get expected cell perception
        maze::Cell expected_cell = this->maze->getCell(candidate.row, candidate.column);

        // check if it still matches observation //TODO: make sure rotation is correct
        if (expected_cell == observation.rotate(candidate.orientation)) {
            new_candidates.push_back(candidate);
        }

    }

    this->candidates = new_candidates;
}
