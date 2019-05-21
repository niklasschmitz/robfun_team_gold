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
    return fmod(theta + (2.0 * M_PI), (2.0 * M_PI));
}

inline double absMin(double value, double min){
    if (fabs(value) < min) {
        value = min * sgn(value);
    }
    return value;
}


#endif //SRC_TOOLS_H
