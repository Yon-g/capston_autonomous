/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter/particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 100; //set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    // Normal distribution for x, y and theta
    normal_distribution<double> dist_x(x, std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

    // create particles and set their values
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;

        particles[i] = p;
        weights[i] = p.weight;
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    default_random_engine gen;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i]; // get address of particle to update
        double new_x, new_y, new_theta;
        if (yaw_rate != 0) {
            new_x = p->x + (velocity / yaw_rate) * (sin(p->theta + yaw_rate * delta_t) - sin(p->theta));
            new_y = p->y + (velocity / yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate * delta_t));
            if(p->theta + yaw_rate * delta_t >3.14){
                new_theta = p->theta + yaw_rate * delta_t - 6.28;
            }
            else if(p->theta + yaw_rate * delta_t < -3.14){
                new_theta = p->theta + yaw_rate * delta_t + 6.28;
            }
            else{
                new_theta = p->theta + yaw_rate * delta_t;
            }
            
        } else {
            new_x = p->x + velocity * delta_t * cos(p->theta);
            new_y = p->y + velocity * delta_t * sin(p->theta);
            new_theta = p->theta;
        }
        // std::cout<< "new_x :" << new_x<<std::endl;
        // std::cout<< "new_y :" << new_y<<std::endl;
        // std::cout<< "new_z :" << new_theta<<std::endl;
        // add Gaussian Noise to each measurement
        // Normal distribution for x, y and theta
        normal_distribution<double> dist_x(new_x, std_x);
        normal_distribution<double> dist_y(new_y, std_y);
        normal_distribution<double> dist_theta(new_theta, std_theta);

        // update the particle attributes
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);

        // std::cout << "px :"<< p->x;
        // std::cout << "py :"<< p->y;
        // std::cout << "ph :"<< p->theta << std::endl;
        
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
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
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i];
        double wt = 1.0;
        // std::cout << p->x << p->y << std::endl;
        // convert observation from vehicle's to map's coordinate system
        for(int j=0; j<observations.size(); ++j){
            LandmarkObs current_obs = observations[j];
            LandmarkObs transformed_obs;
            
            transformed_obs.x = (current_obs.x * cos(p->theta)) - (current_obs.y * sin(p->theta)) + p->x;
            transformed_obs.y = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;
            transformed_obs.id = current_obs.id;

            // find the predicted measurement that is closest to each observed measurement and assign
            // the observed measurement to this particular landmark
            Map::single_landmark_s landmark;
            double distance_min = std::numeric_limits<double>::max();
            int q = 0;
            for(int k=0; k<map_landmarks.landmark_list.size(); ++k){
                Map::single_landmark_s cur_l = map_landmarks.landmark_list[k];
                double distance = dist(transformed_obs.x, transformed_obs.y, cur_l.x_f, cur_l.y_f);
                
                if(distance < distance_min){
                    distance_min = distance;
                    landmark = cur_l;
                    q = k;
                }
            }
            // if(distance_min>10){
            //     std::cout <<"p->x:" << p->x << ",p->y :" << p->y << ",p->heading :" << p->theta << std::endl;
            //     std::cout <<"ob_x: " <<  current_obs.x << ",ob_y :" << current_obs.y << std::endl;
                
            //     std::cout <<"tr_x: " <<  transformed_obs.x << ",cur_x :" << map_landmarks.landmark_list[q].x_f << std::endl;
            //     std::cout <<"tr_y: " <<  transformed_obs.y << ",cur_y :" << map_landmarks.landmark_list[q].y_f << std::endl;
            // }
            // std::cout <<"distance_min:" <<  distance_min << std::endl;

            // update weights using Multivariate Gaussian Distribution
            // equation given in Transformations and Associations Quiz
            double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f)*3/100,2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y_f)*3/100, 2) / pow(std_y, 2)));
            double denom = 2 * M_PI * std_x * std_y;
            // std::cout <<"num/denom:" <<  num/denom << std::endl;
            wt *= num/denom;

        }
        weights_sum += wt;
        p->weight = wt;
    }
    // std::cout <<"observations.size():" <<  observations.size() << std::endl;

    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        // std::cout << "total_weight :" << weights_sum<< std::endl;
        // std::cout << "p->weight :" << p->weight<< std::endl;
        // std::cout << "p->weight/weight_sum :" << p->weight<< std::endl;

        weights[i] = p->weight;
    }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;

    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }

    particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}


std::pair<std::vector<double>,std::vector<double>> ParticleFilter::estimate(){
    vector<double> mean(3,0.0);
    vector<double> var(3,0.0);

    double total_weight = std::accumulate(weights.begin(),weights.end(),0.0);
    // for(int i = 0; i<num_particles; i++){
    //     Particle *p = &particles[i];
    //     std::cout << "px :"<< p->x << std::endl;
    //     std::cout << "py :"<< p->y << std::endl;
    //     std::cout << "ph :"<< p->theta << std::endl;
    // }
    for(int i = 0; i<num_particles; i++){
        Particle *p = &particles[i];
        mean[0] += p->x * weights[i];
        // std::cout << "p->x_w :"<< p->x <<","<<weights[i] << std::endl;
        mean[1] += p->y * weights[i];
        mean[2] += p->theta * weights[i];
    }

    mean[0] /= total_weight;
    mean[1] /= total_weight;
    mean[2] /= total_weight;

    for(int i = 0; i<num_particles; i++){
        Particle *p = &particles[i];
        var[0] += weights[i]*(p->x - mean[0])*(p->x - mean[0]);
        var[1] += weights[i]*(p->y - mean[1])*(p->x - mean[1]);
        var[2] += weights[i]*(p->theta - mean[2])*(p->x - mean[2]);
    }

    var[0] /= total_weight;
    var[1] /= total_weight;
    var[2] /= total_weight;

    return make_pair(mean,var);
}