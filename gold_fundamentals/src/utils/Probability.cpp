#include "Probability.h"

#include <stdlib.h>
#include <math.h>

double Probability::randomNrBetween0and1() {
    double rand_nr = ((double) rand() / (RAND_MAX));
    return rand_nr;
}

double Probability::gaussian( double x, double std, double mean ) {
	return exp( -(x - mean)*(x - mean) / (2.0*(std * std)));
}

// samples from a gaussian distribution
double Probability::gaussianRandom( double mean, double std ) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	return mean + std * z;
} 

// samples from a uniform distribution
double Probability::uniformRandom(double min, double max) {
	return min + (rand() / (double)RAND_MAX) * (max - min);
}

// returns the shortest difference and direction to go from a to b
double Probability::diffAngle(double a, double b) {
	return normalizeTheta(b - a);
}

// returns an angle between -PI and PI
double Probability::normalizeTheta(double theta) {

  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}
