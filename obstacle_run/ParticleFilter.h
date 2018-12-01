/*
 *   ParticleFilter.h
 *
 *   Author: Goutham
 *
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include "HELPER_FUNCTIONS.H"

struct Particle
{
	int id;
	double x;
	double y;
	double theta;
	double weight;
}

class ParticleFilter {
	int numParticles;
	bool isInit ;
	std::vector<double> weights;

	public:
		void init(double x, double y, double theta, double std[]);
		void prediction(double delta_time, double std_pos[], double velocity, double yaw_rate);

}
#endif