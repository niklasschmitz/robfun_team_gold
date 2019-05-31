#include "DiscreteLocalizer.h"

DiscreteLocalizer::DiscreteLocalizer() {}

DiscreteLocalizer::~DiscreteLocalizer() {}

// map = map from the node [[[T,L,R], ... ]]]
void DiscreteLocalizer::convertMsgGridToMap(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    // find out max horizontal, vertical spread of the map (x=max_nr_of_cols, y=max_nr_of_rows)

    int n_rows = msg_grid->rows.size();
    int n_cols = n_rows ? msg_grid->rows[0].cells.size() : 0;

    // create new Cell array
    maze::Maze* map = new maze::Maze(n_rows, n_cols);

    // for every coodinate
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
            for (int direction = 0; direction < 4; ++direction) {
                double x = (row + 0.5) * maze->CELL_SIDE_LENGTH;
                double y = (col + 0.5) * maze->CELL_SIDE_LENGTH;
                this->candidates.push_back(RobotConfiguration(x, y, (double) direction * M_PI_2));
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
        maze::Cell expected_cell = this->maze->getCell(candidate.position);

        // check if it still matches observation //TODO: make sure rotation is correct
        if (expected_cell == observation.rotate((int) (candidate.theta / M_PI_2))) {
            new_candidates.push_back(candidate);
        }

    }

    this->candidates = new_candidates;
}
