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
    //T_LINE line = linear_regression(xy[0], xy[1]);
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

        std::vector <T_POINT2D> samples;
        samples.push_back(coordinates[rand1]);
        samples.push_back(coordinates[rand2]);

        T_LINE proposed_line = linear_regression(samples);
        //TODO: what happens if the line is perfectly vertical? maybe use vectors?
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

double GridPerceptor::distBetweenLineAndPoint(T_LINE line, T_POINT2D point) {
    // TODO: clean up by using vector arithmetic implicitly

    // construct normal vector from line direction
    T_POINT2D normal;
    normal.x = line.u.y;
    normal.y = - line.u.x;

    // construct difference of support vector x0 and point
    T_POINT2D diff_x0_point;
    diff_x0_point.x = x0.x - point.x;
    diff_x0_point.y = x0.y - point.y;

    // project difference onto normal vector to get distance
    double dist = diff_x0_point.x * normal.x + diff_x0_point.y * normal.y;

    return dist;
}

bool GridPerceptor::testLineSimilarity(std::vector <T_LINE> lines, T_LINE line) {
    // TODO implement

    // tested all lines and didnt find a similar one
    return false;
}