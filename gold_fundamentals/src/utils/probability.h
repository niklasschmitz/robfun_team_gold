#ifndef SRC_PROBABILITY_H
#define SRC_PROBABILITY_H


namespace probability {

    // evaluates the gaussian distribution at x
    static double gaussian(double x, double std, double mean = 0);

    // samples from a gaussian distribution
    static double gaussianRandom(double mean, double std);

    // samples from a uniform distribution
    static double uniformRandom(double min, double max);

    static double diffAngle(double a, double b);

    static double normalizeTheta(double theta);

};


#endif //SRC_PROBABILITY_H
