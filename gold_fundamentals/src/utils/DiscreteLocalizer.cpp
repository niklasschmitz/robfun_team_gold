#include "DiscreteLocalizer.h"

DiscreteLocalizer::DiscreteLocalizer() {
    maze = new maze::Maze();
    ros::NodeHandle n;
    update_map = true;
    received_map = false;
    map_sub = n.subscribe("map", 1, &DiscreteLocalizer::mapCallback, this);
}

DiscreteLocalizer::~DiscreteLocalizer() {}

void DiscreteLocalizer::mapCallback(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    if (update_map) {
        convertMsgGridToMap(msg_grid);
        update_map = false;
        received_map = true;
    }
}

// TODO: Subscribe to mapPublisher
// map = map from the node [[[T,L,R], ... ]]]
void DiscreteLocalizer::convertMsgGridToMap(const gold_fundamentals::Grid::ConstPtr &msg_grid) {

    int n_rows = msg_grid->rows.size();
    int n_cols = n_rows ? msg_grid->rows[0].cells.size() : 0;

    maze->map.clear();

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

gold_fundamentals::Pose DiscreteLocalizer::actWithCandidate(gold_fundamentals::Pose candidate, int action) {
    // action:  0 - move forward one cell
    //          1 - turn left and move forward
    //          2 - turn 180 degrees and move forward
    //          3 - turn right and move forward

    gold_fundamentals::Pose new_candidate = candidate;
    new_candidate.orientation += action;
    new_candidate.orientation %= 4;

    // from here on the movement is with respect to the global frame
    if (new_candidate.orientation == 0) {
        new_candidate.column++; // move one cell to the right
    } else if (new_candidate.orientation == 1) {
        new_candidate.row--; // move one cell up
    } else if (new_candidate.orientation == 2) {
        new_candidate.column--; // move one cell to the left
    } else {
        new_candidate.row++; // move one cell down
    }

    return new_candidate;
}

void DiscreteLocalizer::estimateConfiguration(int action, maze::Cell observation) {
    std::vector<gold_fundamentals::Pose> new_candidates;

    for (int i = 0; i < this->candidates.size(); ++i) {

        // project old candidate into future position according to discrete motion model
        gold_fundamentals::Pose new_candidate = actWithCandidate(candidates[i], action);

        // discard if new_candidate is not within bounds of the maze anymore
        if (new_candidate.row < 0 || new_candidate.row >= maze->n_rows
            || new_candidate.column < 0 || new_candidate.column >= maze->n_rows) {
            continue;
        }

        // get expected cell perception
        maze::Cell expected_cell = this->maze->getCell(new_candidate.row, new_candidate.column);

        // check if it still matches observation
        if (expected_cell == observation.rotate(new_candidate.orientation)) {
            new_candidates.push_back(new_candidate);
        }

    }

    this->candidates = new_candidates;
}
