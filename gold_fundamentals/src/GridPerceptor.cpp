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

    std::vector <T_CARTESIAN_COORD> coordinates;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        double theta = msg->angle_min + msg->angle_increment * i; //might be angle_max - increment * i
        double radius = msg->ranges[i];

        if (!isnan(radius)) {  // only consider non-nan points
            T_CARTESIAN_COORD coord = convertPolarToCartesian(theta, radius);
            coordinates.push_back(coord);
        }
    }

    std::vector<T_LINE> lines = ransac(coordinates);
    if(lines.size() == 0) {
        ROS_INFO("no line in sight");
    } else {
        for(int i=0; i<lines.size(); ++i) {
            ROS_INFO("line %d, alpha %lf, beta %lf", i, lines[i].alpha, lines[i].beta);
        }
    }
    //ROS_INFO("alpha %lf, beta %lf", line.alpha, line.beta);
}


T_CARTESIAN_COORD GridPerceptor::convertPolarToCartesian(double theta, double radius) {
    T_CARTESIAN_COORD coord;
    coord.x = radius * std::cos(theta);
    coord.y = radius * std::sin(theta);

    return coord;
}


T_LINE GridPerceptor::linear_regression(std::vector <T_CARTESIAN_COORD> coordinates) {
    // assume (x, y)'s in cartesian coordinates.
    // regress y given x. model: y = alpha + beta*x

    // compute means
    double x_mean = 0;
    double y_mean = 0;
    for (int i = 0; i < coordinates.size(); ++i) {
        x_mean += coordinates[i].x;
        y_mean += coordinates[i].y;
    }
    x_mean /= coordinates.size();
    y_mean /= coordinates.size();

    // compute covariance of (x,y) and variance of x
    double cov_xy = 0;
    double var_x = 0;
    double std_xi; //tmp variable
    for (int i = 0; i < coordinates.size(); ++i) {
        cov_xy += (coordinates[i].x - x_mean) * (coordinates[i].y - y_mean);
        std_xi = coordinates[i].x - x_mean;
        var_x += std_xi * std_xi;
    }

    // compute alpha and beta
    double beta = cov_xy / var_x;
    double alpha = y_mean - beta * x_mean;

    T_LINE line;
    line.alpha = alpha;
    line.beta = beta;

    return line;
}

std::vector <T_LINE> GridPerceptor::ransac(std::vector <T_CARTESIAN_COORD> coordinates) {
    // how often do we generate a random sample
    int iterations = 10;

    // number of points that have to be within the epsilon so the sample is qualified as a line
    // +2 as the 2 samples will always be inside
    int inliers_threshold = 10 + 2;

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

        std::vector <T_CARTESIAN_COORD> samples;
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

double GridPerceptor::distBetweenLineAndPoint(T_LINE line, T_CARTESIAN_COORD point) {
    // make sure not to divide by zero and create M87*
    if (line.alpha == 0) {
        line.alpha = 1e-6;
    }

    double p2_x = (point.y + 1 / line.alpha - line.beta) / (line.alpha + 1 / line.alpha);
    double p2_y = line.alpha * p2_x + line.beta;

    double dist_x = std::abs(p2_x - point.x);
    double dist_y = std::abs(p2_y - point.y);
    double dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));

    return dist;
}

bool GridPerceptor::testLineSimilarity(std::vector <T_LINE> lines, T_LINE line) {

    // if the dist between the angles of the lines is greater than the dist_threshold -> not similar
    double angle_threshold = 5;

    // the dist_threshold is tested if the alpha threshold defines the lines as similar
    // the dist_threshold how far the lines are apart from each other. if they are close -> similar
    double dist_threshold = 5;

    for(int i=0; i<lines.size(); ++i){
        if(std::abs(std::atan2(lines[i].alpha,1) - std::atan2(line.alpha,1)) > dist_threshold) {
            continue;
        } else {
            // lines have alpha_diff smaller than the threshold.
            // now the dist_threshold is tested

            // determine avg_alpha to be able to calculate dist between two parallel lines
            double avg_alpha = (lines[i].alpha + line.alpha) / 2;

            // take a point on the first line and calculate dist from that point to the other line
            T_CARTESIAN_COORD point1;
            point1.x = 0;
            point1.y = line.alpha * point1.x + line.beta;

            double dist = distBetweenLineAndPoint(lines[i], point1);
            if (dist < dist_threshold) {
                // they are close together and have the 'same' alpha -> similar
                return true;
            }
        }
    }

    // tested all lines and didnt find a similar one
    return false;
}