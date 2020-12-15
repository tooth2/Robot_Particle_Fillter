#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "particle_filter.h"
#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

     num_particles = 100;  // TODO: Set the number of particles
    std::default_random_engine gen;
    
    // This line creates a random normal (Gaussian) noise distribution for x, y, theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    //Initialize all particles to first position and all weights to 1.
    //Add random Gaussian noise to each particle.
    for (int i=0; i<num_particles; i++) {     
        
        // Sample from these normal distributions like this:
        //   sample_x = dist_x(gen);
        //   where "gen" is the random engine initialized earlier.
        Particle p ;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
      
        // Set of current particles
	  	particles.push_back(p);
      	weights.push_back(1);

    }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate)  {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    std::default_random_engine gen;

  
    for(auto &p: particles){
    	double new_x ;
    	double new_y ;
    	double new_theta;
        if(fabs(yaw_rate)<0.00001){
            new_x = p.x+velocity*delta_t*cos(p.theta);
            new_y = p.y+velocity*delta_t*sin(p.theta);
            new_theta = p.theta;
            
        }
        else{
          //update based on equation
            new_x = p.x+ velocity/yaw_rate*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta));
            new_y = p.y+ velocity/yaw_rate*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t));
            new_theta = p.theta+yaw_rate * delta_t;
        }
        
        // add a random normal (Gaussian) noise distribution for x, y, theta to each measurement
        std::normal_distribution<double> dist_x(new_x, std_pos[0]);
        std::normal_distribution<double> dist_y(new_y, std_pos[1]);
        std::normal_distribution<double> dist_theta(new_theta, std_pos[2]);
        //update the particle attributes
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
    }
}


void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(auto pred : predicted){
      double dist_min = std::numeric_limits<double>::max();
      for(auto observation : observations){
        double distance = dist(observation.x, observation.y, pred.x, pred.y); // distance b/w obs and landmark
        if(distance < dist_min){
          observation.id = pred.id;
        }
        dist_min = distance;
      }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double weights_sum = 0;

    for(auto &p: particles){
        double wt = 1.0;

        // convert observation from vehicle's to map's coordinate system
        for(auto observation : observations){
            LandmarkObs current_obs = observation;
            LandmarkObs transformed_obs;

            transformed_obs.x = (current_obs.x * cos(p.theta)) - (current_obs.y * sin(p.theta)) + p.x;
            transformed_obs.y = (current_obs.x * sin(p.theta)) + (current_obs.y * cos(p.theta)) + p.y;
            transformed_obs.id = current_obs.id;

            // find the predicted measurement that is closest to each observed measurement and assign
            // the observed measurement to this particular landmark
            Map::single_landmark_s landmark;
            double distance_min = std::numeric_limits<double>::max();

            for(int k=0; k<map_landmarks.landmark_list.size(); ++k){
                Map::single_landmark_s cur_landmark = map_landmarks.landmark_list[k];
                double distance = dist(transformed_obs.x, transformed_obs.y, cur_landmark.x_f, cur_landmark.y_f);
                if(distance < distance_min){
                    distance_min = distance;
                    landmark = cur_landmark;
                }
            }

            // update weights using Multivariate Gaussian Distribution
            // equation given in Transformations and Associations Quiz
            double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f), 2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y_f), 2) / pow(std_y, 2)));
            double denom = 2 * M_PI * std_x * std_y;
            wt *= num/denom;
        }
        weights_sum += wt;
        p.weight = wt;
    }
    //particles[i].weight = 1
    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }

}

void ParticleFilter::resample(){
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::default_random_engine gen;

    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }

    particles = resampled_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
