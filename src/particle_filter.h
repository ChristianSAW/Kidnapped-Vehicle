/**
 * particle_filter.h
 * 2D particle filter class.
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};


class ParticleFilter {  
 public:
  // Constructor
  // @param num_particles Number of particles
  ParticleFilter() : num_particles(0), is_initialized(false) {}

  // Destructor
  ~ParticleFilter() {}

  // Cases Related to updating weights
  //int CASE_1 = 0;
  //int CASE_A = 1;
  //int CASE_A_a = 1;    
  //int CASE_A_b = 0;
  //int CASE_B = 0;

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity, 
                  double yaw_rate);

  
  /**
   * nearestNeighbor_multiAss uses a nearest neighbors approach to find 
   * which observations correspond to which landmarks. Allows for a single
   * predicted landmark to be associated with multiple observed landmarks. 
   *
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void nearestNeighbor_multiAss(std::vector<LandmarkObs> predicted, 
                                     std::vector<LandmarkObs>& observations);

  /**
   * nearestNeighbor_singleAss uses a nearest neighbors approach to find 
   * which observations correspond to which landmarks. Allows for a single
   * predicted landmark to be associated with only a single observed landmark. 
   *
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void nearestNeighbor_singleAss(std::vector<LandmarkObs> predicted, 
                                     std::vector<LandmarkObs>& observations);
  
  /**
   * dataAssociation Finds which observations correspond to which landmarks 
   *   (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void dataAssociation(std::vector<LandmarkObs> predicted, 
                       std::vector<LandmarkObs>& observations);
  

  /**
   * compID is a comparator function which sorts in increasing order based off 
   * Landmark id. 
   *
   * @param a id of Landmark a
   * @param b id of Landmark b
   */
  static bool compID(LandmarkObs a, LandmarkObs b);

  /**
   * calcWeightSameSize calclates and returns the weight for a particle given 
   * observation and prediction measurements. Assumes a given prediction measurement 
   * can be paird with a single observation measurement. 
   *
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observationsT Vector of landmark observations
   * @param predicted Vector of predicted landmark observations
   */
  double calcWeightSameSize(double std_landmark[],
                        std::vector<LandmarkObs> &observationsT,
                        std::vector<LandmarkObs> &predicted);

  /**
   * calcWeightDiffSize calclates and returns the weight for a particle given 
   * observation and prediction measurements. Assumes a given prediction measurement 
   * can be paird with multiple observation measurements. 
   *
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observationsT Vector of landmark observations
   * @param predicted Vector of predicted landmark observations
   */
  double calcWeightDiffSize(double std_landmark[],
                        std::vector<LandmarkObs> &observationsT,
                        std::vector<LandmarkObs> &predicted);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[], 
                     const std::vector<LandmarkObs> &observations,
                     const Map &map_landmarks);
  
  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */

  void resample();

  /**
   * Set a particles list of associations, along with the associations'
   *   calculated world x,y coordinates
   * This can be a very useful debugging tool to make sure transformations 
   *   are correct and assocations correctly connected
   */
  void SetAssociations(Particle& particle, const std::vector<int>& associations,
                       const std::vector<double>& sense_x, 
                       const std::vector<double>& sense_y);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }

  /**
   * Used for obtaining debugging information related to particles.
   */
  std::string getAssociations(Particle best);
  std::string getSenseCoord(Particle best, std::string coord);
  void printParticles(std::ofstream &outfile);
  void printIDs(std::vector<LandmarkObs> landmarks);
  void printObs(std::vector<LandmarkObs> landmarks);
  void printInit(double x, double y, double theta, double std[]);
  void printPartW();
  void updateWeightsTS(double sensor_range, double std_landmark[],
		                   const std::vector<LandmarkObs> &observations, 
                       const Map &map_landmarks);

  // Set of current particles
  std::vector<Particle> particles;

 private:
  // Number of particles to draw
  int num_particles; 
  
  // Flag, if filter is initialized
  bool is_initialized;
  
  // Vector of weights of all particles
  std::vector<double> weights; 
};

#endif  // PARTICLE_FILTER_H_