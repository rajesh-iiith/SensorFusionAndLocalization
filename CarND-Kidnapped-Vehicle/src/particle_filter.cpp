/*
 * particle_filter.cpp
*/

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

 using std::normal_distribution;
 using std::default_random_engine;
 using vector_t = std::vector<double>;
 default_random_engine gen;
 using namespace std;


 void ParticleFilter::init(double x, double y, double theta, double std[]) 
 {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

 // Create normal distribution for x, y, theta
 	normal_distribution<double> dist_x(x, std[0]);
 	normal_distribution<double> dist_y(y, std[1]);
 	normal_distribution<double> dist_theta(theta, std[2]);

 	double sample_x, sample_y, sample_psi;

 	num_particles = 100;
 	weights.resize(num_particles, 1.0f);

 	for(unsigned i=0; i<num_particles; i++)
 	{
 		Particle p;
 		p.x = dist_x(gen);
 		p.y = dist_y(gen);
 		p.theta = dist_theta(gen);
 		p.id = i;
 		p.weight = 1.0f;
 		particles.push_back(p);
 	}
 	is_initialized = true;
 }

 void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
 {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

 	normal_distribution<double> noise_x(0.0, std_pos[0]);
 	normal_distribution<double> noise_y(0.0, std_pos[1]);
 	normal_distribution<double> noise_theta(0.0, std_pos[2]);

 	if (fabs(yaw_rate) < 0.0001) {
 		yaw_rate = 0.0001;
 	}

 	for (auto&& particle : particles){

 		particle.x += (velocity / yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta)) + noise_x(gen);
 		particle.y += (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t)) + noise_y(gen);
 		particle.theta += yaw_rate * delta_t + noise_theta(gen);
 	}

 }

 void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
 {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

 	for (int i = 0; i < observations.size(); i++) {

 		int current_j;
 		double current_smallest_error = INFINITY;

 		for (int j = 0; j < predicted.size(); j++) {

 			const double dx = predicted[j].x - observations[i].x;
 			const double dy = predicted[j].y - observations[i].y;
 			const double error = dx * dx + dy * dy;

 			if (error < current_smallest_error) {
 				current_j = j;
 				current_smallest_error = error;
 			}
 		}
 		observations[i].id = current_j;
 	}

 }

 const LandmarkObs LocalToGlobalMapping(const LandmarkObs& obs, const Particle& p)
 {
 	LandmarkObs out;

  // First rotate the local coordinates to the right orientation
 	out.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
 	out.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
 	out.id = obs.id;
 	return out;
 }


 inline const double get2dGaussian(const LandmarkObs& obs, const LandmarkObs &lm, const double sigma[])
 {
 	auto cov_x = sigma[0]*sigma[0];
 	auto cov_y = sigma[1]*sigma[1];
 	auto normalizer = 2.0*M_PI*sigma[0]*sigma[1];
 	auto dx = (obs.x - lm.x);
 	auto dy = (obs.y - lm.y);
 	return exp(-(dx*dx/(2*cov_x) + dy*dy/(2*cov_y)))/normalizer;
 }

 void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
 	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
 {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

	for(unsigned p_ctr=0; p_ctr < particles.size(); p_ctr++)
	{
		auto p = particles[p_ctr];

		std::vector<LandmarkObs> predicted_landmarks;

		for(auto lm : map_landmarks.landmark_list)
		{
			LandmarkObs lm_pred;
			lm_pred.x = lm.x_f;
			lm_pred.y = lm.y_f;
			lm_pred.id = lm.id_i;
			auto dx = lm_pred.x - p.x;
			auto dy = lm_pred.y - p.y;

	      // Add only if in range
			if(dx*dx + dy*dy <= sensor_range*sensor_range)
				predicted_landmarks.push_back(lm_pred);
		}
		std::vector<LandmarkObs> transformed_obs;
		double total_prob = 1.0f;

	    // transform coordinates of all observations (for current particle)
		for(auto obs_lm : observations)
		{
			auto obs_global = LocalToGlobalMapping(obs_lm, p);
			transformed_obs.push_back(std::move(obs_global));
		}
	    // Stores index of associated landmark in the observation
		dataAssociation(predicted_landmarks, transformed_obs);

		for(unsigned i=0; i < transformed_obs.size(); i++)
		{
			auto obs = transformed_obs[i];
	      // Assume sorted by id and starting at 1
			auto assoc_lm = predicted_landmarks[obs.id];

			double pdf = get2dGaussian(obs, assoc_lm, sigma_landmark);
			total_prob *= pdf;
		}
		particles[p_ctr].weight = total_prob;
		weights[p_ctr] = total_prob;
	}
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::discrete_distribution<int> d(weights.begin(), weights.end());
	std::vector<Particle> new_particles;

	for(unsigned i = 0; i < num_particles; i++)
	{
		auto ind = d(gen);
		new_particles.push_back(std::move(particles[ind]));
	}
	particles = std::move(new_particles);

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
