/* NOTE: Some of the code was developed by some group members during another course and was integrated here */

#include "ParticleFilter.h"
#include "Probability.h"

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

ParticleFilter::ParticleFilter() {
    ros::NodeHandle n;
    map_sub = n.subscribe("map", 1, &ParticleFilter::mapCallback, this);
    updatemap_service = n.advertiseService("update_map", &ParticleFilter::setUpdateMap, this);
    this->update_map = true;

    this->numberOfParticles = 1000;
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

    init();
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
}

void ParticleFilter::mapCallback(const gold_fundamentals::Grid::ConstPtr &msg_grid) {
    if(update_map) {
//        ROS_INFO("map callback %d", msg_grid->rows[0].cells[0].walls[0]);
//        ROS_INFO("map callback %d", msg_grid->rows[0].cells[1].walls.size());
        oc_grid.convertMsgGridToOccupancyGrid(msg_grid, inverse_resolution);
        oc_grid.printGrid();
        //oc_grid.publishToRviz();
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
		for (int h=0; h < this->likelihoodFieldHeight; h++) {
			int idx = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, w, h);

			double z_hit = 1.0-zRand;
			//generate normal distribution with mean at the nearest obstacle
			double p_hit = Probability::gaussian(distMap[idx], sigmaHit_scaled, 0);

			double z_rand = zRand;
			double p_rand = 1.0;

			this->likelihoodField[idx] = log(z_hit * p_hit + z_rand * p_rand);

			//this->likelihoodField[idx] = z_hit * p_hit + z_rand * p_rand; //Only needed for plotting the Likelihood Field
		}
	}

	/*//Only needed for plotting the Likelihood Field
	double max = 0;
	double min = 0;
	for(int i=0; i < map.info.width; i++)
	{
		for(int j=0; j < map.info.height; j++)
		{
			max = fmax(max, likelihoodField[i + j * likelihoodFieldWidth]);
			min = fmin(min, likelihoodField[i + j * likelihoodFieldWidth]);
		}
	}

	for(int i=0; i < map.info.width; i++)
	{
		for(int j=0; j < map.info.height; j++)
		{
			likelihoodField[i + j * likelihoodFieldWidth] = (likelihoodField[i + j * likelihoodFieldWidth]-min)/max;
		}
	}*/

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
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
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
		int nr_of_ranges = 511;

		//particle postions are now needed in [pixel], therefore divison by resolution
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
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

