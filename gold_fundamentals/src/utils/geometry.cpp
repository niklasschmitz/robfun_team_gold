#include "geometry.h"

double distBetweenLineAndPoint(const T_LINE &line, const T_POINT2D &point) {
    // construct normal vector from line direction
    T_POINT2D normal;
    normal.x = line.u.y;
    normal.y = -line.u.x;

    // construct difference of support vector x0 and point
    T_POINT2D diff_x0_point = line.x0 - point;

    // project difference onto normal vector to get distance
    double dist = diff_x0_point * normal;

    return std::fabs(dist);
}