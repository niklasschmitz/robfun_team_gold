#include "GridPerceptor.h"


GridPerceptor::GridPerceptor() {
    ros::NodeHandle n;
    sub_laser = n.subscribe("scan_filtered", 1, &GridPerceptor::laserCallback, this);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

GridPerceptor::~GridPerceptor() {
}

void GridPerceptor::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    //ROS_INFO("GridPerceptor laserCallback");
    //ROS_INFO("%i", msg->ranges.size());
    //ROS_INFO("min deg %f", msg->angle_min);
    //ROS_INFO("min deg %f", msg->angle_increment);
    //ROS_INFO("%f", msg->ranges[msg->ranges.size() / 2]);
    std::vector<T_POINT2D> coordinates;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        double theta = msg->angle_min + msg->angle_increment * i; //might be angle_max - increment * i
        double radius = msg->ranges[i];

        if (!isnan(radius)) {  // only consider non-nan points
            T_POINT2D coord = convertPolarToCartesian(theta, radius);
            coordinates.push_back(coord);
        }
    }

    if (coordinates.size() != 0) {
        std::vector<T_RATED_LINE> lines = ransac(coordinates);
        if (lines.size() == 0) {
            //ROS_INFO("no line in sight");
        } else {
            //ROS_INFO("nr_of_lines %d", lines.size());
            for (int i = 0; i < lines.size(); ++i) {
                //ROS_INFO("line %d, x0 %lf, u %lf", i+1, lines[i].x0, lines[i].u);
            }
        }
        publishLines(lines);
    }
    //ROS_INFO("nr of lines %d" , static_cast<int>(lines.size()));
    //ROS_INFO("alpha %lf, beta %lf", line.alpha, line.beta);
}


T_POINT2D GridPerceptor::convertPolarToCartesian(double theta, double radius) {
    T_POINT2D coord;
    coord.x = radius * std::cos(theta);
    coord.y = radius * std::sin(theta);

    return coord;
}


std::vector<T_RATED_LINE> GridPerceptor::ransac(std::vector<T_POINT2D> coordinates) {
    // how often do we generate a random sample
//    ROS_INFO("---- RANSAC ----");
    int iterations = 50; // dont put too low, otherwise we don't find short edges because we miss them when sampling

    // number of points that have to be within the epsilon so the sample is qualified as a line
    // +2 as the 2 samples will always be inside
    int inliers_threshold = 20 + 2;

    // boundary around the line. samples within are inliers, others are outliers
    double epsilon = 0.005;

    std::vector<T_RATED_LINE> lines;

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
            // test for all point if it is in epsilon range (so we know total nr of inliers later)
            if (dist < epsilon) {
                nr_of_inliers += 1;
                //ROS_INFO("nr of inliers = %d", nr_of_inliers);
            }
        }

        // test if we broke the threshold
        if (nr_of_inliers >= inliers_threshold) {
            T_RATED_LINE rated_line = {proposed_line, nr_of_inliers};
            // see if a similar line exists already: yes -> ignore, no -> add to response
            bool similar_exists = testLineSimilarity(lines, rated_line);
            if (!similar_exists) {
//                ROS_INFO("adding line");
                lines.push_back(rated_line);
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

    return std::fabs(dist);
}

bool GridPerceptor::testLineSimilarity(std::vector<T_RATED_LINE> &lines, T_RATED_LINE rated_line) {
    // TODO implement

    // if the angle between the lines is greater than the angle_threshold -> not similar
    double angle_threshold = 45 * M_PI / 180.0;

    // the dist_threshold is tested if the alpha threshold defines the lines as similar
    // it is measured how far the lines are apart from each other. if they are close -> similar
    double dist_threshold = 0.3;

    for (int i = 0; i < lines.size(); ++i) {
        double angle = T_POINT2D::angleBetweenVectors(lines[i].line.u, rated_line.line.u);
        //ROS_INFO("angle before %lf", angle);
        //ROS_INFO("nr of lines %d", static_cast<int>(lines.size()));
        angle = fmod(std::fabs(angle),
                     M_PI); // fmod = float modulo, %M_PI as vectors facing in the opposite direction are a similar line
        //ROS_INFO("angle after %lf", angle);
        if (angle < angle_threshold || angle > M_PI - angle_threshold) {
            // ROS_INFO("__rejected line (similar)");
//            return true;
            // angles are similar, test distance
            // calc dist between line and support vector, direction used is the one of the already existing line
            double dist = distBetweenLineAndPoint(lines[i].line, rated_line.line.x0);
            if (dist < dist_threshold) {
                // same angle, close to each other -> similar
                // now test which line has the better fit
                if (lines[i].inliers < rated_line.inliers) {
                    // the new line has more inliers
                    // delete the old line, move in the other line
                    lines[i] = rated_line;
                }
                //ROS_INFO("similar");
                return true;
            } else {
                //ROS_INFO("unsimilar, dist %lf", dist);
            }
        }
        //ROS_INFO("____accepted line, angle %lf", angle);
    }

    // tested all lines and didnt find a similar one
    return false;
}

void GridPerceptor::publishLines(std::vector<T_RATED_LINE> lines) {
    visualization_msgs::Marker line_list; // line_list is a term from ros

    line_list.header.frame_id = "laser"; // right?
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.03;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.lifetime = ros::Duration();

    for (int i = 0; i < lines.size(); ++i) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        T_LINE cur_line = lines[i].line;
        p1.x = cur_line.x0.x - 20 * cur_line.u.x;
        p1.y = cur_line.x0.y - 20 * cur_line.u.y;
        p2.x = cur_line.x0.x + 20 * cur_line.u.x;
        p2.y = cur_line.x0.y + 20 * cur_line.u.y;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }

    // Publish the marker
    if (marker_pub) {
        if (marker_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the marker");
        } else {
            if (!line_list.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
                marker_pub.publish(line_list);
            }
        }
    } else {
        ROS_INFO("marker pub invalid");
    }

}