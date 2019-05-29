/* NOTE: Some of the code was developed by some group members during another course and was integrated here */

#include "ParticleFilter.h"
#include "Probability.h"
#include <visualization_msgs/Marker.h>

//#include "tf/tf.h"

//using namespace std;

//ParticleFilter::ParticleFilter(int numberOfParticles) {
//	this->numberOfParticles = numberOfParticles;
//
//	// initialize particles
//	for (int i = 0; i <this->numberOfParticles; i++) {
//		this->particleSet.push_back(new Particle());
//	}
//
//	// this variable holds the estimated robot pose
//	this->bestHypothesis = new Particle();
//
//	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
//	this->laserSkip = 5;
//
//	// distance map used for computing the likelihood field
//	this->distMap = NULL;
//}

#define VISUALIZE_LIKELIHOODMAP 0

ParticleFilter::ParticleFilter() {
    ros::NodeHandle n;
    map_sub = n.subscribe("map", 1, &ParticleFilter::mapCallback, this);
    maze_pub = n.advertise<visualization_msgs::Marker>("maze_visualization", 10);
    likelihoodMap_pub = n.advertise<visualization_msgs::Marker>("likelihoodmap_visualization", 10);
    distMap_pub = n.advertise<visualization_msgs::Marker>("distancemap_visualization", 10);
    allParticles_pub = n.advertise<visualization_msgs::Marker>("allparticles_visualization", 10);
    bestParticle_pub = n.advertise<visualization_msgs::Marker>("bestparticle_visualization", 10);
    updatemap_service = n.advertiseService("update_map", &ParticleFilter::setUpdateMap, this);
    this->update_map = true;

    this->numberOfParticles = 2000;
    this->inverse_resolution = 100; // pixel / meter

    // initialize particles
    for (int i = 0; i <this->numberOfParticles; i++) {
        this->particleSet.push_back(new Particle());
    }

    // this variable holds the estimated robot pose
    this->bestHypothesis = new Particle();

    // at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
    this->laserSkip = 5;

    // distance map used for computing the likelihood field
    this->distMap = NULL;

    this->odomAlpha1 = 0.2;
    this->odomAlpha2 = 0.2;
    this->odomAlpha3 = 0.2;
    this->odomAlpha4 = 0.2;

    this->minLikelihood = 0.8;
    this->standardDev = 0.3;

    this->initialized = false;

}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i <this->numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;

}

void ParticleFilter::init() {
    this->setMotionModelOdometry(odomAlpha1, odomAlpha2, odomAlpha3, odomAlpha4);
    this->setMeasurementModelLikelihoodField(oc_grid, minLikelihood, standardDev);
    this->initParticlesUniform();
    this->initialized = true;
}

void ParticleFilter::mapCallback(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    if(update_map) {
//        ROS_INFO("map callback %d", msg_grid->rows[0].cells[0].walls[0]);
//        ROS_INFO("map callback %d", msg_grid->rows[0].cells[1].walls.size());
        initialized = false;
        oc_grid.convertMsgGridToOccupancyGrid(msg_grid, inverse_resolution);
        oc_grid.printGrid();
        init();
        //printDistanceMap();
        //printLikelihoodMap();
        publishOcGridToRviz();
        //publishDistanceMapToRviz();
        publishAllParticlesToRviz();
#if VISUALIZE_LIKELIHOODMAP == 1
        publishLikelihoodMapToRviz();
#endif
        update_map = false;
    }
}


bool ParticleFilter::setUpdateMap(gold_fundamentals::UpdateMap::Request &req, gold_fundamentals::UpdateMap::Response &res) {
    update_map = true;
    res.success = true;
    return true;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

//generate uniformly distributed particles
void ParticleFilter::initParticlesUniform() {
    	//get map properties
    	int mapWidth, mapHeight;
    	double mapResolution;
    	this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	for (int i = 0; i <this->numberOfParticles; i++) {
		// particle position needs to be expressed in [meter] while the mapWidth and mapHeight are given in [pixel]
		// therefore the dimensions have to be multiplied by the resolution [meter/pixel]
		this->particleSet[i]->x = Probability::uniformRandom(0,mapWidth*mapResolution);
		this->particleSet[i]->y = Probability::uniformRandom(0,mapHeight*mapResolution);
		this->particleSet[i]->theta = Probability::uniformRandom(-M_PI,M_PI);
		this->particleSet[i]->weight = 1.0 / this->getNumberOfParticles();
	}
	this->sumOfParticleWeights = 1.0;
}

//generate normal distributed particles
void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
	for (int i = 0; i <this->numberOfParticles; i++) {
		this->particleSet[i]->x = Probability::gaussianRandom(mean_x, std_xx);
		this->particleSet[i]->y = Probability::gaussianRandom(mean_y, std_yy);
		this->particleSet[i]->theta = Probability::gaussianRandom(mean_theta, std_tt);
		this->particleSet[i]->weight = 1.0 / this->getNumberOfParticles();
	}
	this->sumOfParticleWeights = 1.0;
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.height * map.width];
	this->likelihoodFieldWidth = map.width;
	this->likelihoodFieldHeight = map.height;
	this->likelihoodFieldResolution = 1.0/map.inverse_resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	double sigmaHit_scaled = sigmaHit / this->likelihoodFieldResolution;
	for (int w = 0; w < this->likelihoodFieldWidth; w++) {
        for (int h = 0; h < this->likelihoodFieldHeight; h++) {
            int idx = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, w, h);

            double z_hit = 1.0 - zRand;
            //generate normal distribution with mean at the nearest obstacle
            double p_hit = Probability::gaussian(distMap[idx], sigmaHit_scaled, 0);

            double z_rand = zRand;
            double p_rand = 1.0;
#if VISUALIZE_LIKELIHOODMAP == 0
			this->likelihoodField[idx] = log(z_hit * p_hit + z_rand * p_rand);
        }
    }
#else
			this->likelihoodField[idx] = z_hit * p_hit + z_rand * p_rand; //Only needed for plotting the Likelihood Field
		}
	}

	//Only needed for plotting the Likelihood Field
	double max = 0;
	double min = 1;
	for(int i=0; i < map.width; i++)
	{
		for(int j=0; j < map.height; j++)
		{
			max = fmax(max, likelihoodField[i + j * likelihoodFieldWidth]);
			min = fmin(min, likelihoodField[i + j * likelihoodFieldWidth]);
		}
	}

	for(int i=0; i < map.width; i++)
	{
		for(int j=0; j < map.height; j++)
		{
			likelihoodField[i + j * likelihoodFieldWidth] = (likelihoodField[i + j * likelihoodFieldWidth]-min)/(max-min);
		}
	}
#endif
	ROS_INFO("...DONE creating likelihood field!");

}

void ParticleFilter::calculateDistanceMap(const OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.width; x++) {
		for (int y = 0; y < map.height; y++) {
			if (map.grid_data[x + y * map.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.grid_data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.grid_data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border) {
						    // TODO why was there a bugfix necessary here (line below was missing)
						    if(x+i >= 0 && y+j >= 0 && x+i < likelihoodFieldWidth && y+j < likelihoodFieldHeight) {
                                distMap[x + i + (y + j) * likelihoodFieldWidth] = 0.0;
                            }
						}
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
	this->sumOfParticleWeights = 0.0;

	for (int i = 0; i <this->numberOfParticles; i++) {
		double weight = 0.0;
//		int nr_of_ranges = 511;
        // REVIEW: check if this is correct!
        int nr_of_ranges = laserScan->ranges.size()-1;
//        ROS_INFO("%d", nr_of_ranges);

        //particle positions are now needed in [pixel], therefore divison by resolution
		double particleX = this->particleSet[i]->x / this->likelihoodFieldResolution;
		double particleY = this->particleSet[i]->y / this->likelihoodFieldResolution;
		double particleTheta = this->particleSet[i]->theta;

		for (int laser_nr = 0; laser_nr <= nr_of_ranges; laser_nr+=laserSkip) {

			//get values of that laser scan
			double range = laserScan->ranges[laser_nr];
			double laser_angle = -laserScan->angle_min + laserScan->angle_increment * laser_nr;

			double map_angle = Probability::normalizeTheta(laser_angle + particleTheta);

			//laser endpoint
			int laser_hit_x_map = (int)particleX - (int)(cos(map_angle) * range / this->likelihoodFieldResolution);
			int laser_hit_y_map = (int)particleY - (int)(sin(map_angle) * range / this->likelihoodFieldResolution);

			int idx = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight,
				laser_hit_x_map, laser_hit_y_map);

			//Check whether the endpoint is inside the map. If not use a lower prob then the min prob inside the map
			if (laser_hit_x_map < 0 || laser_hit_y_map < 0 || laser_hit_x_map >= likelihoodFieldWidth || laser_hit_y_map >= likelihoodFieldHeight)
			{
				weight += log(7e-1);
			}
			else
			{
				weight += this->likelihoodField[idx];
			}
		}

		double tr_weight = exp(weight);

		this->sumOfParticleWeights += tr_weight;
		this->particleSet[i]->weight = tr_weight;
	}

	//normalize the weights
	for (int i = 0; i < this->numberOfParticles; i++) {
		if(this->sumOfParticleWeights != 0) {
			this->particleSet[i]->weight = this->particleSet[i]->weight / this->sumOfParticleWeights;
		} else {
			ROS_WARN("Weight norm factor is zero");
		}
	}
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	//validate input (first iteration has NaN in it)
	if(std::isnan(oldX) || std::isnan(oldY) || std::isnan(oldTheta) ||
			std::isnan(newX) || std::isnan(newY) || std::isnan(newTheta)) {
		ROS_WARN("Not a valid value");
		return;
	}

	//determine odometry information
	double delta_trans = sqrt( pow(newX-oldX,2) + pow(newY-oldY,2) );
	double delta_rot1 = Probability::diffAngle(oldTheta, atan2( newY-oldY, newX-oldX));
	double delta_rot2 = Probability::diffAngle(delta_rot1, Probability::diffAngle(oldTheta, newTheta));


	for (int i = 0; i <this->numberOfParticles; i++) {
		//add uncertainty
		double d_rot1_hat = Probability::gaussianRandom(delta_rot1,
			this->odomAlpha1 * abs(delta_rot1) + this->odomAlpha2 * delta_trans);

		double d_trans_hat = Probability::gaussianRandom(delta_trans,
			this->odomAlpha3 * delta_trans + this->odomAlpha4 * (abs(Probability::normalizeTheta(delta_rot1 + delta_rot2))));

		double d_rot2_hat = Probability::gaussianRandom(delta_rot2,
			this->odomAlpha1 * abs(delta_rot2) + this->odomAlpha2 * delta_trans);

		//assign new states
		this->particleSet[i]->x = this->particleSet[i]->x + d_trans_hat * cos(this->particleSet[i]->theta + d_rot1_hat);
		this->particleSet[i]->y = this->particleSet[i]->y + d_trans_hat * sin(this->particleSet[i]->theta + d_rot1_hat);
		this->particleSet[i]->theta = Probability::normalizeTheta(Probability::normalizeTheta(this->particleSet[i]->theta + d_rot1_hat) + d_rot2_hat);
	}
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	int nofP = this->getNumberOfParticles();
	std::vector<Particle> pSetNew;
	pSetNew.reserve(nofP);

	double r = Probability::uniformRandom(0, 1.0/nofP);
	double u = 0;
	double c = this->particleSet[0]->weight;
	int i = 0;
	double maxWeight = 0;

	for(int j = 0; j < nofP; j++)
	{
		u = r + (j) * 1.0/nofP;
		while(u > c)
		{
			i = i + 1;
			c = c + this->particleSet[i]->weight;
		}
		Particle part(this->particleSet[i]->x,this->particleSet[i]->y,this->particleSet[i]->theta,0);
		if(maxWeight < this->particleSet[i]->weight)
		{
			maxWeight = this->particleSet[i]->weight;
			this->bestHypothesis->x = this->particleSet[i]->x;
			this->bestHypothesis->y = this->particleSet[i]->y;
			this->bestHypothesis->theta = this->particleSet[i]->theta;
			this->bestHypothesis->weight = this->particleSet[i]->weight;
		}
		pSetNew[j] = part;
	}

	//replace the old particles with the the new ones
	for(int i = 0; i < nofP; i++)
	{
		this->particleSet[i]->x = pSetNew[i].x;
		this->particleSet[i]->y = pSetNew[i].y;
		this->particleSet[i]->theta = pSetNew[i].theta;
		this->particleSet[i]->weight = this->getNumberOfParticles();
	}

    publishBestParticleToRviz();
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

void ParticleFilter::publishOcGridToRviz() {

    visualization_msgs::Marker points;

    points.header.frame_id = "maze"; // right?
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 1.0;
    points.scale.y = 1.0;
    points.scale.z = 1.0;
    points.color.r = 1.0;
    points.color.a = 1.0;
    points.lifetime = ros::Duration();

    // iterate over the pixels of the occupancy grid and create rviz markers
    for (int row = 0; row < oc_grid.height; row++) {
        for (int col = 0; col < oc_grid.width; col++) {
            // see if there is an obstacle
            if (oc_grid.grid_data[col + row * oc_grid.width] != 0) {
                geometry_msgs::Point p;
                p.x = col;
                p.y = row;
//                p.y = row;
                p.z = 0;

                points.points.push_back(p);
            }
        }
    }

    // Publish the marker
    if (maze_pub) {
        if (maze_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the maze pub");
        } else {
            if (!points.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
                ROS_INFO("publishing map");
                maze_pub.publish(points);
            }
        }
    } else {
        ROS_INFO("maze pub invalid");
    }
}

void ParticleFilter::publishDistanceMapToRviz() {

    visualization_msgs::Marker distMapPoints;

    distMapPoints.header.frame_id = "maze";
    distMapPoints.header.stamp = ros::Time::now();
    distMapPoints.ns = "points_and_lines";
    distMapPoints.action = visualization_msgs::Marker::ADD;
    distMapPoints.pose.orientation.w = 1.0;
    distMapPoints.id = 1;
    distMapPoints.type = visualization_msgs::Marker::POINTS;

    // scale
    distMapPoints.scale.x = 1.0;
    distMapPoints.scale.y = 1.0;
    distMapPoints.scale.z = 1.0;

    // color
    distMapPoints.color.r = 0.0;
    distMapPoints.color.g = 1.0;
    distMapPoints.color.b = 0.0;
    distMapPoints.color.a = 1.0;

    distMapPoints.lifetime = ros::Duration();

    // iterate over the pixels of the occupancy grid and create rviz markers
    for (int row = 0; row < oc_grid.height; row++) {
        for (int col = 0; col < oc_grid.width; col++) {
            // create a box for every point in the likelihood field
            geometry_msgs::Point p;
            p.x = col;
            p.y = row;
            p.z = distMap[col + row*oc_grid.width];

            distMapPoints.points.push_back(p);
        }
    }

    // Publish the marker
    if (distMap_pub) {
        if (distMap_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the distance map pub");
        } else {
            if (!distMapPoints.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
                ROS_INFO("publishing distMap");
                distMap_pub.publish(distMapPoints);
            }
        }
    } else {
        ROS_INFO("distMap pub invalid");
    }
}


void ParticleFilter::publishLikelihoodMapToRviz() {

    visualization_msgs::Marker likelihoodPoints;

    likelihoodPoints.header.frame_id = "maze"; // right?
    likelihoodPoints.header.stamp = ros::Time::now();
    likelihoodPoints.ns = "points_and_lines";
    likelihoodPoints.action = visualization_msgs::Marker::ADD;
    likelihoodPoints.pose.orientation.w = 1.0;
    likelihoodPoints.id = 2;
    likelihoodPoints.type = visualization_msgs::Marker::POINTS;

    // scale
    likelihoodPoints.scale.x = 1.0;
    likelihoodPoints.scale.y = 1.0;
    likelihoodPoints.scale.z = 1.0;

    // color
    likelihoodPoints.color.r = 0.3;
    likelihoodPoints.color.g = 0.3;
    likelihoodPoints.color.b = 1.0;
    likelihoodPoints.color.a = 1.0;

    likelihoodPoints.lifetime = ros::Duration();

    // iterate over the pixels of the occupancy grid and create rviz markers
    for (int row = 0; row < oc_grid.height; row++) {
        for (int col = 0; col < oc_grid.width; col++) {
            // create a box for every point in the likelihood field
            geometry_msgs::Point p;
            p.x = col;
            p.y = row;
//            p.z = pow(M_E, likelihoodField[col + row*oc_grid.width])*20;
            p.z = (likelihoodField[col + row*oc_grid.width])*30;

            likelihoodPoints.points.push_back(p);
        }
    }

    // Publish the marker
    if (likelihoodMap_pub) {
        if (likelihoodMap_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the likelihoodMap pub");
        } else {
            if (!likelihoodPoints.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
                ROS_INFO("publishing likelihoodMap (scaled)");
                likelihoodMap_pub.publish(likelihoodPoints);
            }
        }
    } else {
        ROS_INFO("likelihoodMap pub invalid");
    }
}

RvizParticleVisualisation ParticleFilter::getRvizParticleVisualisation(const Particle &particle) {
    RvizParticleVisualisation part_vis;
    part_vis.x_pos = particle.x * inverse_resolution;
    part_vis.y_pos = particle.y * inverse_resolution;
    double indicator_length = 0.01;
    part_vis.direction_indicator_x = (particle.x + indicator_length * cos(particle.theta)) * inverse_resolution;
    part_vis.direction_indicator_y = (particle.y + indicator_length * sin(particle.theta)) * inverse_resolution;

    return part_vis;
}

void ParticleFilter::publishAllParticlesToRviz() {
    //-----position-----
    visualization_msgs::Marker allParticlesPositionMarker;

    allParticlesPositionMarker.header.frame_id = "maze";
    allParticlesPositionMarker.header.stamp = ros::Time::now();
    allParticlesPositionMarker.ns = "points_and_lines";
    allParticlesPositionMarker.action = visualization_msgs::Marker::ADD;
    allParticlesPositionMarker.pose.orientation.w = 1.0;
    allParticlesPositionMarker.id = 3;
    allParticlesPositionMarker.type = visualization_msgs::Marker::POINTS;

    // scale
    allParticlesPositionMarker.scale.x = 3.0;
    allParticlesPositionMarker.scale.y = 3.0;
    allParticlesPositionMarker.scale.z = 3.0;

    // color
    allParticlesPositionMarker.color.r = 0.3;
    allParticlesPositionMarker.color.g = 1.0;
    allParticlesPositionMarker.color.b = 0.5;
    allParticlesPositionMarker.color.a = 1.0;

    allParticlesPositionMarker.lifetime = ros::Duration();

    //-----direction------
    visualization_msgs::Marker allParticlesDirectionMarker;

    allParticlesDirectionMarker.header.frame_id = "maze";
    allParticlesDirectionMarker.header.stamp = ros::Time::now();
    allParticlesDirectionMarker.ns = "points_and_lines";
    allParticlesDirectionMarker.action = visualization_msgs::Marker::ADD;
    allParticlesDirectionMarker.pose.orientation.w = 1.0;
    allParticlesDirectionMarker.id = 4;
    allParticlesDirectionMarker.type = visualization_msgs::Marker::POINTS;

    // scale
    allParticlesDirectionMarker.scale.x = 2.5;
    allParticlesDirectionMarker.scale.y = 2.5;
    allParticlesDirectionMarker.scale.z = 2.5;

    // color
    allParticlesDirectionMarker.color.r = 0.3;
    allParticlesDirectionMarker.color.g = 0.3;
    allParticlesDirectionMarker.color.b = 1.0;
    allParticlesDirectionMarker.color.a = 1.0;

    allParticlesDirectionMarker.lifetime = ros::Duration();

    for(int part_idx=0; part_idx<numberOfParticles; part_idx++) {
        RvizParticleVisualisation part_vis = getRvizParticleVisualisation(particleSet[part_idx]);

        geometry_msgs::Point p;
        p.x = part_vis.x_pos;
        p.y = part_vis.y_pos;
        p.z = 0;
        allParticlesPositionMarker.points.push_back(p);

        //TODO display line to indicate direction
        geometry_msgs::Point p_dir;
        p_dir.x = part_vis.direction_indicator_x;
        p_dir.y = part_vis.direction_indicator_y;
        p_dir.z = 0;
        allParticlesDirectionMarker.points.push_back(p_dir);
    }

    // Publish the marker
    if (allParticles_pub) {
        if (allParticles_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the allParticles pub");
        } else {
            if (!allParticlesPositionMarker.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
//                ROS_INFO("publishing allPosParticles");
                allParticles_pub.publish(allParticlesPositionMarker);
            }
            if (!allParticlesDirectionMarker.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
//                ROS_INFO("publishing allDirParticles");
                allParticles_pub.publish(allParticlesDirectionMarker);
            }
        }
    } else {
        ROS_INFO("bestParticle pub invalid");
    }
}

void ParticleFilter::publishBestParticleToRviz() {
    visualization_msgs::Marker bestParticleMarker;

    bestParticleMarker.header.frame_id = "maze"; // right?
    bestParticleMarker.header.stamp = ros::Time::now();
    bestParticleMarker.ns = "points_and_lines";
    bestParticleMarker.action = visualization_msgs::Marker::ADD;
    bestParticleMarker.pose.orientation.w = 1.0;
    bestParticleMarker.id = 5;
    bestParticleMarker.type = visualization_msgs::Marker::POINTS;

    // scale
    bestParticleMarker.scale.x = 5.0;
    bestParticleMarker.scale.y = 5.0;
    bestParticleMarker.scale.z = 5.0;

    // color
    bestParticleMarker.color.r = 1.0;
    bestParticleMarker.color.g = 1.0;
    bestParticleMarker.color.b = 1.0;
    bestParticleMarker.color.a = 1.0;

    bestParticleMarker.lifetime = ros::Duration();

    geometry_msgs::Point p;
    p.x = bestHypothesis->x*inverse_resolution;
    p.y = bestHypothesis->y*inverse_resolution;
    p.z = 0;//likelihoodField[col + row*oc_grid.width];

    bestParticleMarker.points.push_back(p);

    // Publish the marker
    if (bestParticle_pub) {
        if (bestParticle_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the bestParticle pub");
        } else {
            if (!bestParticleMarker.points.empty()) {
                // only publish if we have data (preventing sigsegv?)
//                ROS_INFO("publishing bestParticle");
                bestParticle_pub.publish(bestParticleMarker);
            }
        }
    } else {
        ROS_INFO("bestParticle pub invalid");
    }
}

void ParticleFilter::printDistanceMap() {
    ROS_INFO("PRINTING DISTANCE MAP");
    for( int row=0; row<oc_grid.height; row++) {
        std::ostringstream row_string;
        //row_string << row+1;
        for( int col=0; col<oc_grid.width; col++) {
            int val = distMap[row * oc_grid.width + col];
            if(val > 9) {
                row_string << val;
            } else {
                row_string << 0;
                row_string << val;
            }
            row_string << ',';
        }

        ROS_INFO("%s", row_string.str().c_str());
    }
}

void ParticleFilter::printLikelihoodMap() {
    ROS_INFO("PRINTING LIKELIHOOD MAP");
    for( int row=0; row<oc_grid.height; row++) {
        std::ostringstream row_string;
        //row_string << row+1;
        for( int col=0; col<oc_grid.width; col++) {
#if VISUALIZE_LIKELIHOODMAP == 0
            double val = likelihoodField[row * oc_grid.width + col];
            val = pow(M_E, val); //convert log likelihood to likelihood
#else
            double val = likelihoodField[row * oc_grid.width + col];
#endif
            row_string << val;
            row_string << ',';
        }

        ROS_INFO("%s", row_string.str().c_str());
    }
}
