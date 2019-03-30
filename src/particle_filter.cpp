/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::min_element;

using namespace std;

// Cases Related to updating weights
int CASE_1 = 0;
int CASE_A = 1;
int CASE_A_a = 1;    
int CASE_A_b = 0;
int CASE_B = 0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  // std = [sig_x, sig_y, sig_theta]
  std::default_random_engine gen;           // random generator
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // POPULATE PARTICLES VECTOR
  // Determine later if we want to normalize weights
  for(int i = 0; i < num_particles; ++i) {
    Particle p; 
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
    particles.push_back(p);
  }
  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // std_pos = [x_var, y_var, theta_var]; process noise.
  
  // initializations:
  double Xf, Yf, Thetaf;
  std::default_random_engine gen;           // random generator
  
  // Prediction step for each particle i in particles
  for(Particle i : particles) {
    Xf = i.x + (velocity/yaw_rate)*(sin(i.theta + yaw_rate*delta_t)-sin(i.theta));
    Yf = i.y + (velocity/yaw_rate)*(cos(i.theta) - cos(i.theta + yaw_rate*delta_t));
    Thetaf = i.theta + yaw_rate*delta_t;
  
    // Add noise
    normal_distribution<double> dist_x(Xf, std_pos[0]);
    normal_distribution<double> dist_y(Yf, std_pos[1]);
    normal_distribution<double> dist_theta(Thetaf, std_pos[2]);
  
    // POPULATE PARTICLES VECTOR
    i.x = dist_x(gen);
    i.y = dist_y(gen);
    i.theta = dist_theta(gen);
  }
  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // can modify predicted vector. Will not change actual vector. 
  #if (CASE_1)
  vector<double> distV;
  for(LandmarkObs i : observations) {
    distV.clear();                      // clear vector for each loop
    for(LandmarkObs j : predicted) {
      distV.push_back(dist(i.x,i.y,j.x,j.y));
    }
    // get index of closest landmark
    int minIndex = min_element(distV.begin(),distV.end())-distV.begin(); 
    i.id = predicted[minIndex].id;                  // pair pred and obs id
    predicted.erase(predicted.begin()+minIndex);    // erase pared landmark from pred list
  }
  #endif
  #if (CASE_A)
  #if (CASE_A_a) // allow predictions to be associated with more than 1 observation
  double min_dist = numeric_limits<double>::max(); // initialize min_dist to max possible val
  int minInd;
  double dist_;
  for (LandmarksObs i : observations) {
    minInd = -1;
    for(LandmarksObs j : predicted) {
      dist_ = dist(i.x,i.y,j.x,j.y);
      if ( dist_ < min_dist) {
        min_dist = dist_;
        min
      }
    }
    
  }
  #endif 
  #endif
  
}

bool comp(LandmarkObs a, LandmarkObs b) {
  return a.id < b.id;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  //* Notes:
  // [1] map has landmarks in order of id, so no need to sort landmarks in prediction vector by 
  // id as you populate in order of increasing id. Only need to sort observations vector.
  // [2] std_landmark = [sig_x, sig_y]
  
  #if (CASE_1)
  // Variables
  vector<LandmarkObs> predicted;
  double Xr;
  double Yr;
  double Xob, Yob, Xpr, Ypr;
  double SigX = std_landmark[0];
  double SigY = std_landmark[1];
  
  // need to make a copy of observations so we can modify it. 
  // alternatively, we could 1) change dataAssociation to change predicted and not observations id.
  // Problem is that we cant output anything so we have to change observations. 
  // 2) swich predicted and observations input into dataAssociation.
  vector<LandmarkObs> observationsT = observations; 

  // updating weight for each particle
  for(Particle i : particles) {
    // Calculate prediction vector 
    predicted.clear();
    // Populate predictions in increasing landmark iD order (Transform Global -> Robot)
    for(int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
      Xr = cos(i.theta)*i.x + sin(i.theta)*i.y + map_landmarks.landmark_list[j].x_f;
      Yr = cos(i.theta)*i.y - sin(i.theta)*i.x + map_landmarks.landmark_list[j].y_f;
      
      LandmarkObs lm;
      lm.id = map_landmarks.landmark_list[j].id_i;
      lm.x = Xr;
      lm.y = Yr;
      
      predicted.push_back(lm);
    }
    
    // Calculate dataAssociation between prediction and observation vector
    dataAssociation(predicted, observationsT);
    
    // Calculate weight as product sum of probability for each measurement pair (Z*,Z)
    // [1] Sort observationsT by ID so index i == j (sort in increasing id order)
    // sort(predicted.begin(),predicted.end(),comp);          // sort predicted (not necessary)
    sort(observationsT.begin(),observationsT.end(),comp);  // sort observations 
    
    // [2] Set up variables 
    double W = 1;
    double Prob;
    
    // [3] Loop Through each observation, prediction pair and calculate prob + update product sum
    for( int j = 0; j < observationsT.size(); ++j) {
      Xob = observationsT[j].x;
      Yob = observationsT[j].y;
      Xpr = predicted[j].x;
      Ypr = predicted[j].y;
      Prob = (1/(2*M_PI*SigX*SigY))*exp(-(((Xob-Xpr)*(Xob-Xpr)/(2*SigX*SigX))+((Yob-Ypr)*(Yob-Ypr)/(2*SigY*SigY))));
      W = W*Prob;
    }
    
    // Update i.weight
    i.weight = W;
  }
  #endif
  
  #if (CASE_A)
  
  #endif
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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