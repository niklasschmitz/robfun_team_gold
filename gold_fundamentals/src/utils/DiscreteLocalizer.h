#ifndef SRC_DISCRETELOCALIZER_H
#define SRC_DISCRETELOCALIZER_H

#include "Maze.h"
#include "geometry.h"
#include "gold_fundamentals/Grid.h"
#include "gold_fundamentals/Pose.h"

using namespace maze;

/**
 * The DiscreteLocalizer knows the map of the maze. That includes the connectivity
 * as well as the scale (given by the side length of a single cell).
 * It keeps  a set of possible configurations ('candidates') which are
 * consistent with previous cell observations of the environment
 */
class DiscreteLocalizer {
public:
    Maze* maze;

    // set of possible states consistent with history
    std::vector<gold_fundamentals::Pose> candidates;

    DiscreteLocalizer();
    ~DiscreteLocalizer();

    /**
     * @param action  0 - move forward one cell
     *                1 - turn left and move forward
     *                2 - turn 180 degrees and move forward
     *                3 - turn right and move forward
     * @param observation  a Cell indicating in what directions walls are perceived.
     *                     Note: this is relative to the robot
     */
    void estimateConfiguration(int action, maze::Cell observation);

    void populateCandidates();

    void convertMsgGridToMap(const gold_fundamentals::Grid_<std::allocator<void> >::ConstPtr &msg_grid);

    gold_fundamentals::Pose actWithCandidate(gold_fundamentals::Pose candidate, int action);
};


#endif //SRC_DISCRETELOCALIZER_H
