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
     *
     * @param action  acts like a delta in config space.
     *                will be added to each previous configuration.
     * @param observation  a Cell indicating in what directions walls are perceived.
     *                     Note: this is relative to the robot
     */
    void estimateConfiguration(gold_fundamentals::Pose action, maze::Cell observation);

    void populateCandidates();

    void convertMsgGridToMap(const gold_fundamentals::Grid_<std::allocator<void> >::ConstPtr &msg_grid);
};


#endif //SRC_DISCRETELOCALIZER_H
