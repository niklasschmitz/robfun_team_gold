#ifndef PROBABILITY_H
#define PROBABILITY_H

class Probability {

public:
    static double randomNrBetween0and1();
    static double gaussian( double x, double std, double mean = 0 );
	static double gaussianRandom( double mean, double std );
	static double uniformRandom( double min, double max );
	static double diffAngle( double a, double b );
	static double normalizeTheta( double theta );
};

#endif