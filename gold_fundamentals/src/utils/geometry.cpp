#include "geometry.h"

double distBetweenLineAndPoint(const T_LINE &line, const T_VECTOR2D &point) {
    // construct normal vector from line direction
    T_VECTOR2D normal;
    normal.x = line.u.y;
    normal.y = -line.u.x;

    // construct difference of support vector x0 and point
    T_VECTOR2D diff_x0_point = line.x0 - point;

    // project difference onto normal vector to get distance
    double dist = diff_x0_point * normal;

    return std::fabs(dist);

}

T_VECTOR2D operator*(const double scalar, const T_VECTOR2D vec) {
return T_VECTOR2D(scalar * vec.x, scalar * vec.y);
}

// https://math.stackexchange.com/questions/406864/intersection-of-two-lines-in-vector-form
// https://www.cliffsnotes.com/study-guides/algebra/algebra-ii/linear-sentences-in-two-variables/linear-equations-solutions-using-determinants-with-two-variables
T_VECTOR2D intersectionOfTwoLines(const T_LINE &line1, const T_LINE &line2) {

    // see if lines are parallel
    if(fmod(T_VECTOR2D::angleBetweenVectors(line1.u, line2.u), M_PI) == 0) {
        return T_VECTOR2D(NAN, NAN);
    }

    // cramer's rule to solve the system x0_1 + a * u_1 = x0_2 + b * u_2
    T_MATRIX2D d  = T_MATRIX2D(line1.u, (-1)*line2.u);
    T_VECTOR2D v_x0 = T_VECTOR2D(line2.x0.x - line1.x0.x, line2.x0.y - line1.x0.y);
    T_MATRIX2D dx = T_MATRIX2D(v_x0, (-1)*line2.u);

    double a = dx.determinant()/d.determinant();

    return line1.x0 + a * line1.u;
}