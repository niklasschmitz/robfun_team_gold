#ifndef SRC_DISCRETELOCALIZER_H
#define SRC_DISCRETELOCALIZER_H

#include "Maze.h"
#include "geometry.h"

/**
 * The RobotConfiguration type stores a position vector and
 * an orientation angle theta. It can be thought as an element
 * of the (continuous) configuration space of our robot
 */
struct RobotConfiguration {
    T_VECTOR2D position;
    double theta; // TODO: ensure valid theta in a consistent representation

    // constructor
    RobotConfiguration(double x = 0, double y = 0, double theta = 0) {
        position = T_VECTOR2D(x, y);
        this->theta = theta;
    }

    RobotConfiguration(T_VECTOR2D position, double theta = 0)
            : position(position), theta(theta) {}

    // trivial copy constructor
    RobotConfiguration &operator=(const RobotConfiguration &other) {
        position = other.position;
        theta = other.theta;
    }

    // equality
    bool operator==(const RobotConfiguration &other) {
        return (position == other.position && theta == other.theta);
    }

    // addition
    RobotConfiguration operator+(const RobotConfiguration &other) const {
        return RobotConfiguration(position + other.position, theta + other.theta);
    }

    // subtraction
    RobotConfiguration operator-(const RobotConfiguration &other) const {
        return RobotConfiguration(position - other.position, theta - other.theta);
    }
};


/**
 * The DiscreteLocalizer knows the map of the maze. That includes the connectivity
 * as well as the scale (given by the side length of a single cell).
 * It keeps  a set of possible configurations ('candidates') which are
 * consistent with previous cell observations of the environment
 */
class DiscreteLocalizer {
    maze::Maze map;

    // set of possible states consistent with history
    std::vector<RobotConfiguration> candidates;

    DiscreteLocalizer();
    ~DiscreteLocalizer();

    /**
     *
     * @param action  acts like a delta in config space.
     *                will be added to each previous configuration.
     * @param observation  a Cell indicating in what directions walls are perceived.
     *                     Note: this is relative to the robot
     */
    void estimateConfiguration(RobotConfiguration action, maze::Cell observation);
};


#endif //SRC_DISCRETELOCALIZER_H
