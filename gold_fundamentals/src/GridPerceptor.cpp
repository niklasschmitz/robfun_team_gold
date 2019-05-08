#include "GridPerceptor.h"

GridPerceptor::GridPerceptor() {
    ros::NodeHandle n;
    sub_laser = n.subscribe("scan_filtered", 1, &GridPerceptor::laserCallback, this);
}

GridPerceptor::~GridPerceptor() {
}

void GridPerceptor::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("GridPerceptor laserCallback");
    //ROS_INFO("%i", msg->ranges.size());
    //ROS_INFO("min deg %f", msg->angle_min);
    //ROS_INFO("min deg %f", msg->angle_increment);
    //ROS_INFO("%f", msg->ranges[msg->ranges.size() / 2]);

    std::vector <T_POINT2D> coordinates;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        double theta = msg->angle_min + msg->angle_increment * i; //might be angle_max - increment * i
        double radius = msg->ranges[i];

        if (!isnan(radius)) {  // only consider non-nan points
            T_POINT2D coord = convertPolarToCartesian(theta, radius);
            coordinates.push_back(coord);
        }
    }

    std::vector <T_LINE> lines = ransac(coordinates);
    //ROS_INFO("alpha %lf, beta %lf", line.alpha, line.beta);
}


T_POINT2D GridPerceptor::convertPolarToCartesian(double theta, double radius) {
    T_POINT2D coord;
    coord.x = radius * std::cos(theta);
    coord.y = radius * std::sin(theta);

    return coord;
}


std::vector <T_LINE> GridPerceptor::ransac(std::vector <T_POINT2D> coordinates) {
    // how often do we generate a random sample
    int iterations = 10;

    // number of points that have to be within the epsilon so the sample is qualified as a line
    // +2 as the 2 samples will always be inside
    int inliers_threshold = 20 + 2;

    // boundary around the line. samples within are inliers, others are outliers
    double epsilon = 5;

    std::vector <T_LINE> lines;

    int nr_of_coords = coordinates.size();

    // take random samples for x iterations, see how well it fits
    for (int i = 0; i < iterations; ++i) {
        // generate two random numbers to select two coordinates randomly
        int rand1 = std::rand() % nr_of_coords; // 0 to nr_of_coords-1
        int rand2 = rand1;
        while (rand1 == rand2) { // make sure rand1 != rand2
            rand2 = std::rand() % nr_of_coords;
        }

        T_POINT2D x1 = coordinates[rand1];
        T_POINT2D x2 = coordinates[rand2];

        // construct hypothesis
        T_LINE proposed_line = constructLineParameterForm(x1, x2);

        int nr_of_inliers = 0;

        // test how many inliers the line has
        for (int j = 0; j < coordinates.size(); j++) {
            double dist = distBetweenLineAndPoint(proposed_line, coordinates[j]);

            // test if point is in epsilon range
            if (dist < epsilon) {
                nr_of_inliers += 1;

                // test if we broke the threshold
                if (nr_of_inliers == inliers_threshold) {
                    // see if a similar line exists already: yes -> ignore, no -> add to response
                    bool similar_exists = testLineSimilarity(lines, proposed_line);
                    if (!similar_exists) {
                        lines.push_back(proposed_line);
                    }
                }
            }
        }
    }

    return lines;
}

T_LINE GridPerceptor::constructLineParameterForm(T_POINT2D x1, T_POINT2D x2) {
    T_LINE line;

    // support vector
    line.x0 = x1;

    // construct directional vector u
    T_POINT2D u;
    u.x = x2.x - x1.x;
    u.y = x2.y - x1.y;

    // normalize u
    T_POINT2D::normalize(u);
    line.u = u;

    return line;
}

double GridPerceptor::distBetweenLineAndPoint(T_LINE line, T_POINT2D point) {
    // TODO: clean up by using vector arithmetic implicitly

    // construct normal vector from line direction
    T_POINT2D normal;
    normal.x = line.u.y;
    normal.y = -line.u.x;

    // construct difference of support vector x0 and point
    T_POINT2D diff_x0_point;
    diff_x0_point.x = line.x0.x - point.x;
    diff_x0_point.y = line.x0.y - point.y;

    // project difference onto normal vector to get distance
    double dist = diff_x0_point.x * normal.x + diff_x0_point.y * normal.y;

    return dist;
}

bool GridPerceptor::testLineSimilarity(std::vector <T_LINE> lines, T_LINE line) {
    // TODO implement

    // if the angle between the lines is greater than the angle_threshold -> not similar
    double angle_threshold = 5;

    // the dist_threshold is tested if the alpha threshold defines the lines as similar
    // it is measured how far the lines are apart from each other. if they are close -> similar
    double dist_threshold = 5;

    for( int i = 0; i < lines.size(); ++i) {
        double angle = T_POINT2D::angleBetweenVectors(lines[i].u, line.u);
        angle = fmod(std::abs(angle), 180); // fmod = float modulo, %180 as vectors facing in the opposite direction are a similar line
        if(angle < angle_threshold) {
            // angles are similar, test distance
            // calc dist between line and support vector, direction used is the one of the already existing line
            double dist = distBetweenLineAndPoint(lines[i], line.x0);
            if(dist < dist_threshold) {
                // same angle, close to each other -> similar
                return true;
            }
        }
    }

    // tested all lines and didnt find a similar one
    return false;
}