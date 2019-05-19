#ifndef SRC_TOOLS_H
#define SRC_TOOLS_H

#include <math.h>

int sgn(double val);

inline double angularToLinear(double angular_vel, double radius) {
    return angular_vel * radius;
}

inline double distanceToTime(double distance, double speed, double radius) {
    return distance / angularToLinear(speed, radius);
}

inline double normalizeAngle(double theta){
    return fmod(theta + (M_PI * 2.0), (M_PI * 2.0));
}



#endif //SRC_TOOLS_H
