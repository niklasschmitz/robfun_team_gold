#ifndef SRC_DISCRETELOCALIZER_H
#define SRC_DISCRETELOCALIZER_H

#include "Maze.h"
#include "geometry.h"

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

class DiscreteLocalizer {
    maze::Maze map;

    // set of possible states consistent with history
    std::vector<RobotConfiguration> candidates;

    DiscreteLocalizer();
    void estimateConfiguration();

    void estimateConfiguration(RobotConfiguration action, maze::Cell observation);
};


#endif //SRC_DISCRETELOCALIZER_H
