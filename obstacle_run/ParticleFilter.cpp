#include "ParticleFilter.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
		numParticles = 600;
		weights.resize(numParticles);
		particles.resize(numParticles);
		
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1);
		normal_distribution<double> dist_theta(theta, std[2]);
		
		default_random_engine random;
		
		for(int i =0; i< numParticles; i++)
		{
			Particle p;
			p.id = i;
			p.x = dist_x(random);
			p.y = dist_y(random);
			p.theta = dist_theta(random);
			p.weight = 1/numParticles;
			particles[i] = p;
			weights[i] = p.weight;
		}
		isInit = true;
}

void ParticleFilter::prediction(double time, double std[], double velocity; double yaw)
{
		default_random_engine random;
		
		for(int i = 0 ; i< numParticles; i++)
		{
				Particle *p = &particles[i];
				
				double newX = p->x + (velocity/yaw) * (sin(p->theta +yaw * time) - sin(p->theta));
				double newY = p->y + (velocity/yaw) * (cos(p->theta) - cos(p->theta +yaw * time));
				double newTheta = p->theta + (yaw * time);
				
				normal_distribution<double> dist_x(newX, std[0]);
				normal_distribution<double> dist_y(newY, std[1);
				normal_distribution<double> dist_theta(newTheta, std[2]);
				
				p->x = dist_x(random);
				p->y = dist_y(random);
				p->theta = dist_theta(random);
		}
}

void ParticleFilter::updateweights()
