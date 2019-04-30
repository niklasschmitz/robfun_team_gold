//
// Created by ros on 27.04.19.
//

#ifndef SRC_TOOLS_H
#define SRC_TOOLS_H

int sgn(double val);

inline double angularToLinear(double angular_vel, double radius) {
    return angular_vel * radius;
}

inline double distanceToTime(double distance, double speed, double radius) {
    return distance / angularToLinear(speed, radius);
}



#endif //SRC_TOOLS_H
