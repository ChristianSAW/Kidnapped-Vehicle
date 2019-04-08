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
//int CASE_1 = 0;
//int CASE_A = 1;
//int CASE_A_a = 1;    
//int CASE_A_b = 0;
//int CASE_B = 0;
const double eps = 0.00001;

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
  
  weights.resize(num_particles);
  
  //particles.clear();
  
  // std = [sig_x, sig_y, sig_theta]
  std::default_random_engine gen;           // random generator
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // POPULATE PARTICLES VECTOR
  // Determine later if we want to normalize weights
  for(int i = 0; i < num_particles; ++i) {
    Particle p; 
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
    particles.push_back(p);
  }
  is_initialized = true;
  

  #if (true) // Debugging
  cout<<"Initialized Particles"<<endl;
  cout<<"Number of particles: "<< particles.size() <<endl;
  cout<<"GPS: ["<<x<<", "<<y<<", "<<theta<<"]"<<endl;
  cout<<"STDEV: ["<<std[0]<<", "<<std[1]<<", "<<std[2]<<"]"<<endl; 
  int n = 10; // must be less than num_particles
  cout<<"Printing First "<<n<<" particles"<<endl;
  for(int i = 0; (i < num_particles) && (i < n); ++i) {
    cout<<"Particle "<< particles[i].id <<"; ["<<particles[i].x<<", "<<particles[i].y;
    cout<<"]; W = "<< particles[i].weight<<endl;
  }
  // Add Particles to 'initialization.txt' file
  // create an open file
  ofstream outfile("/home/workspace/CarND-Kidnapped-Vehicle-Project/output_files/initialized_particles.txt");    
  // populate initialization.txt
  outfile<<"Initialized Particles"<<endl;
  outfile<<"Number of particles: "<< particles.size() <<endl;
  outfile<<"GPS: ["<<x<<", "<<y<<", "<<theta<<"]"<<endl;
  outfile<<"STDEV: ["<<std[0]<<", "<<std[1]<<", "<<std[2]<<"]"<<endl; 
  for(int i = 0; i < num_particles; ++i) {
    outfile<<"Particle "<< particles[i].id <<"; ["<<particles[i].x<<", "<<particles[i].y;
    outfile<<"]; W = "<< particles[i].weight<<endl;
  }
  outfile.close();
  #endif
  
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
  //int j = 0;
  std::default_random_engine gen;           // random generator
  
  // Prediction step for each particle i in particles
  for(Particle &i : particles) {

    if (fabs(yaw_rate) < eps) {
      Xf = i.x + (velocity*delta_t)*cos(i.theta);
      Yf = i.y + (velocity*delta_t)*sin(i.theta);
      Thetaf = i.theta;
    } else {
      Xf = i.x + (velocity/yaw_rate)*(sin(i.theta + yaw_rate*delta_t)-sin(i.theta));
      Yf = i.y + (velocity/yaw_rate)*(cos(i.theta) - cos(i.theta + yaw_rate*delta_t));
      Thetaf = i.theta + yaw_rate*delta_t;
    }
    // Add noise
    normal_distribution<double> dist_x(Xf, std_pos[0]);
    normal_distribution<double> dist_y(Yf, std_pos[1]);
    normal_distribution<double> dist_theta(Thetaf, std_pos[2]);
  
    // POPULATE PARTICLES VECTOR
    i.x = dist_x(gen);
    i.y = dist_y(gen);
    i.theta = dist_theta(gen);

    //particles[j].x = dist_x(gen);
    //particles[j].y = dist_y(gen);
    //particles[j].theta = dist_theta(gen);
    //++j;
  }
  
}

void ParticleFilter::nearestNeighbor_multiAss(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations_) {
  /**
   * Computes the nearest neighbor allowing predited landmarks to be associated
   * with multiple observed landmarks. 
   * Note: predicted.size() can be <, >, = observations.size(); 
   */
  
  double min_dist; 
  int minInd;
  double dist_;
  //cout<<min_dist<<endl; //debugging
  for (LandmarkObs &i : observations_) {
    minInd = -1;
    min_dist = numeric_limits<double>::max(); // initialize min_dist to max possible val
    for(unsigned int j = 0; j < predicted.size(); ++j) {
      dist_ = dist(i.x,i.y,predicted[j].x,predicted[j].y);
      if ( dist_ < min_dist) {
        min_dist = dist_;
        minInd = j; 
      }
    }
    #if(true) // DEBUGGING
    if (minInd == -1) {
      cout << "distance calculation error, index still -1"<<endl;
    }
    #endif
    i.id = predicted[minInd].id;
  }                                   
}

void ParticleFilter::nearestNeighbor_singleAss(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Computes the nearest neighbor allowing predited landmarks to be associated
   * with only a single observed landmark
   * REQUIRES: predicted.size() = observations.size(); 
   */ 
  vector<double> distV;
  for(LandmarkObs &i : observations) {
    distV.clear();                      // clear vector for each loop
    for(LandmarkObs &j : predicted) {
      distV.push_back(dist(i.x,i.y,j.x,j.y));
    }
    // get index of closest landmark
    int minIndex = min_element(distV.begin(),distV.end())-distV.begin(); 
    i.id = predicted[minIndex].id;                  // pair pred and obs id
    predicted.erase(predicted.begin()+minIndex);    // erase pared landmark from pred list
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
  // [CASE 1]: Each observation associated with 1 prediction. 
  // Reqires predictions.size() = observations.size().
  // [CASE A]: 
  // [CASE A_a]: Predictions can be associated with more than 1 observation
  // [CASE A_b]: Predictions can be associated with more than 1 observation iff 
  // predictions.size() < observations.size().  
  
  #if(false)
      if (1) {
        cout<<"Predicted Vector:"<<endl;
        printObs(predicted);
        cout<<"Observed Vector:"<<endl;
        printObs(observations);
      }
  #endif

  #if (false) // CASE_1
    nearestNeighbor_singleAss(predicted, observations);
  #endif
  #if (true) // CASE_A
    #if (true) // CASE_A_a
      nearestNeighbor_multiAss(predicted, observations);
    #endif
    #if (false) // CASE_A_b
      //cout<<"PREPROCESSOR DIRECTIVE FAILURE"<<endl; //DEBUGGING
      // Check size:
      if (predicted.size() < observations.size()) {
        nearestNeighbor_multiAss(predicted, observations);
      } else { // predicted.size() >= observations.size()
        nearestNeighbor_singleAss(predicted, observations);
      }
    #endif 
  #endif
  
}

bool ParticleFilter::compID(LandmarkObs a, LandmarkObs b) {
  return a.id < b.id;
}

double ParticleFilter::calcWeightSameSize(double std_landmark[],
                        vector<LandmarkObs> &observationsT,
                        vector<LandmarkObs> &predicted) {
  /**
   * Calculates the weight for a single particle given the observations
   * and predictions vector. Assumes a given prediction measurement can be
   * paird with a single observation measurement. 
   * 
   * REQUIRES: observationsT.size() == predicted.size()
   */
                
  // Variables 
  double Xob, Yob, Xpr, Ypr;
  double SigX = std_landmark[0];
  double SigY = std_landmark[1];
  double varX = pow(SigX,2);
  double varY = pow(SigY,2);
  double Prob, W; 
  double delX2, delY2;
  
  // Calculate weight as product sum of probability for each measurement pair (Z*,Z)
  // [1] Sort observationsT by ID so index i == j (sort in increasing id order)
  // sort(predicted.begin(),predicted.end(),comp);          // sort predicted (not necessary)
  sort(observationsT.begin(),observationsT.end(),compID);  // sort observations 
    
    // [2] Set up variables 
    W = 1;
    
    // [3] Loop Through each observation, prediction pair and calculate prob + update product sum
    for(unsigned int j = 0; j < observationsT.size(); ++j) {
      Xob = observationsT[j].x;
      Yob = observationsT[j].y;
      Xpr = predicted[j].x;
      Ypr = predicted[j].y;      
      delX2 = pow((Xob-Xpr),2);
      delY2 = pow((Yob-Ypr),2);

      //Prob = (1/(2*M_PI*SigX*SigY))*exp(-(((Xob-Xpr)*(Xob-Xpr)/(2*SigX*SigX))+((Yob-Ypr)*(Yob-Ypr)/(2*SigY*SigY))));
      Prob = (1/(2*M_PI*SigX*SigY))*exp(-((delX2/(2*varX))+(delY2/(2*varY))));
      W = W*Prob;
    }
    
    return W;

}

double ParticleFilter::calcWeightDiffSize(double std_landmark[],
                        vector<LandmarkObs> &observationsT,
                        vector<LandmarkObs> &predicted) {
  /**
   * Calculates the weight for a single particle given the observations
   * and predictions vector. Assumes a given prediction measurement can be
   * paird with multiple observation measurements. 
   * 
   */
  
  // Variables 
  double Xob, Yob, Xpr, Ypr;
  double SigX = std_landmark[0];
  double SigY = std_landmark[1];
  double varX = pow(SigX,2);
  double varY = pow(SigY,2);
  double Prob, W; 
  double delX2, delY2;
  int match = 0;
  
  // Calculate weight as product sum of probability for each measurement pair (Z*,Z)
  // [1] Assign initial value to variable W
  W = 1.0;

  // [2] Loop Through each observation, prediction pair and calculate prob + update product sum
  for(unsigned int j = 0; j < observationsT.size(); ++j) {
    match = 0;
    // [3] find prediction observation id match.
    for (unsigned int k = 0; k < predicted.size(); ++k) {
      if (observationsT[j].id == predicted[k].id) {
        Xpr = predicted[k].x;
        Ypr = predicted[k].y;
        match = 1;
      }
    }
    if (match == 0) {
      cout << "Error, Could not match observation " << j << "with a prediction."<<endl;
      Xpr = predicted[0].x;
      Ypr = predicted[0].y;
    }
    Xob = observationsT[j].x;
    Yob = observationsT[j].y;
    delX2 = pow((Xob-Xpr),2);
    delY2 = pow((Yob-Ypr),2);
    
    Prob = (1/(2*M_PI*SigX*SigY))*exp(-((delX2/(2*varX))+(delY2/(2*varY))));
    //(1/(2*M_PI*s_x*s_y)) * exp(-(pow(pre_x-obs_x,2)/(2*var_x)+(pow(pre_y - obs_y,2)/(2*var_y))));
    W = W*Prob;
  }
  //W = 5;
  return W;
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
  
  #if (false) // CASE_1
    // Variables
    vector<LandmarkObs> predicted;
    double Xr, Yr;
    
    // need to make a copy of observations so we can modify it. 
    // alternatively, we could 1) change dataAssociation to change predicted and not observations id.
    // Problem is that we cant output anything so we have to change observations. 
    // 2) swich predicted and observations input into dataAssociation.
    vector<LandmarkObs> observationsT = observations; 

    // updating weight for each particle
    for(Particle &i : particles) {
      // Calculate prediction vector 
      predicted.clear();
      // Populate predictions in increasing landmark iD order (Transform Global -> Robot)
      for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
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
      
      // Update i.weight
      i.weight = calcWeightSameSize(std_landmark, observationsT, predicted);
    }
  #endif
  
  #if (true) // CASE_A
    // Variables
    vector<LandmarkObs> predicted;
    vector<LandmarkObs> transformed_obs;
    double ml_x, ml_y;
    int ml_id;
    //vector<LandmarkObs> observationsT = observations; 
    #if(true)
      int i_c = 0; 
    #endif
  
    #if(false)
      cout<<"Observations Size"<<observationsT.size()<<endl;
      cout<<"Obervations Original IDs: ";
      printIDs(observations);
      cout<<"ObservationsT Copied IDs: ";
      printIDs(observationsT);
    #endif

    for(Particle &i : particles) {
      
      // CREATE AND POPULATE Predicted & Observation Vectors
      predicted.clear();
      transformed_obs.clear();
       
      // [1] populate predicted vector from map landmark positions. 
      for (unsigned int j=0; j < map_landmarks.landmark_list.size(); j++){
          ml_x = map_landmarks.landmark_list[j].x_f;
          ml_y = map_landmarks.landmark_list[j].y_f;
          ml_id = map_landmarks.landmark_list[j].id_i;

          // [2] Check if landmark is in range (use rectangular region).
          if (fabs(ml_x - i.x) <= sensor_range && fabs(ml_y - i.y) <= sensor_range){
              predicted.push_back(LandmarkObs{ml_id, ml_x, ml_y});
          }
      }
      // [3] Populate transformed_obs vector [local to global coordinates]
      vector<LandmarkObs> transformed_obs;
      for (unsigned int j=0; j < observations.size(); j++){
          double t_x = cos(i.theta)*observations[j].x - sin(i.theta)*observations[j].y + i.x;
          double t_y = sin(i.theta)*observations[j].x + cos(i.theta)*observations[j].y + i.y;
          transformed_obs.push_back(LandmarkObs{observations[j].id, t_x, t_y});
      }

      // Debuging 
      #if (false)
      cout<<"Pred Size, particle "<<i_c<<": "<< observationsT.size()<<endl;
      #endif

      #if(false)
        if ((i_c+1)%10 == 0) {
          cout<<"Particle "<<i_c<<" predicted vector:"<<endl;
          printObs(predicted);
          cout<<"Observed Vector:"<<endl;
          printObs(observationsT);
        }
      #endif

      // Calculate dataAssociation between prediction and observation vector
      dataAssociation(predicted, transformed_obs);

      #if(false) // DEBUGGING 
        if ((i_c+1)%10 == 0) {
          cout<<"Predicted IDs: ";
          printIDs(predicted);
          cout<<"Oberved IDs: ";
          printIDs(transformed_obs);
        }
      #endif
    
      #if (true) // CASE_A_a
      // Update i.weight
      i.weight = calcWeightDiffSize(std_landmark,transformed_obs,predicted);
      #endif

      #if (false) // CASE_A_b
      if (predicted.size() == observationsT.size()) {
        // CASE 1;
        i.weight = calcWeightSameSize(std_landmark, observationsT, predicted);
      } else {
        // CASE A_a:
        i.weight = calcWeightDiffSize(std_landmark, observationsT, predicted);
      }
      #endif

      #if(true)
      ++i_c;
      #endif
    }
  #endif

  #if (false) // CASE_B
    updateWeightsTS(sensor_range, std_landmark, observations, map_landmarks);
  #endif
  
  #if(false)
    cout<<"Particles, weights after updating:"<<endl;
    cout<<"[";
    for (int i = 0; i < num_particles; ++i) {
      cout<<"("<<i<<" : "<<particles[i].weight<<")";  
      if (i < (num_particles-1)) {
        cout<<", ";
      }
    }
    cout<<"]"<<endl;
  #endif
}

void ParticleFilter::updateWeightsTS(double sensor_range, double std_landmark[],
		const vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	for (int i = 0; i < num_particles; i++){

        // retrieve pose
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;

        // a vector holding all the predicted particles within the sensor range
        vector<LandmarkObs> predictions;

        // for each map landmark position
        for (unsigned int j=0; j < map_landmarks.landmark_list.size(); j++){
            float lm_x = map_landmarks.landmark_list[j].x_f;
            float lm_y = map_landmarks.landmark_list[j].y_f;
            int lm_id = map_landmarks.landmark_list[j].id_i;

            // consider a rectangular region which makes the computation faster
            if (fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range){
                predictions.push_back(LandmarkObs{lm_id, lm_x, lm_y});
            }
        }
        // transfer coordinates
        vector<LandmarkObs> transformed_obs;
        for (unsigned int j=0; j < observations.size(); j++){
            double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
            double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
            transformed_obs.push_back(LandmarkObs{observations[j].id, t_x, t_y});
        }

        // perform data association function
        dataAssociation(predictions, transformed_obs);

        // init weight
        particles[i].weight = 1.0;

        for (unsigned int j = 0; j < transformed_obs.size(); j++){
            double obs_x, obs_y, pre_x, pre_y;

            obs_x = transformed_obs[j].x;
            obs_y = transformed_obs[j].y;

            int associated_prediction = transformed_obs[j].id;

            // search for the x,y coords of the prediction associated with the current observations
            for (unsigned int k = 0; k < predictions.size(); k++){
                if (predictions[k].id == associated_prediction){
                    pre_x = predictions[k].x;
                    pre_y = predictions[k].y;
                }
            }

            // calculate weight for this observation with multivariate Gaussian
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double var_x = pow(s_x,2);
            double var_y = pow(s_y,2);
            double obs_w = (1/(2*M_PI*s_x*s_y)) * exp(-(pow(pre_x-obs_x,2)/(2*var_x)+(pow(pre_y - obs_y,2)/(2*var_y))));

            particles[i].weight *= obs_w;
        }
	}
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }
  default_random_engine gen;
  // generate random starting index for resampling wheel
  uniform_int_distribution<int> uniintdist(0, num_particles-1);

  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // resample
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

particles = new_particles;

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

void ParticleFilter::printParticles(std::ofstream &outfile) {
  for(unsigned int i = 0; i < particles.size(); ++i) {
    outfile<<"Particle "<< particles[i].id <<"; ["<<particles[i].x<<", "<<particles[i].y;
    outfile<<"]; W = "<< particles[i].weight<<endl;
  }
}

void ParticleFilter::printIDs(vector<LandmarkObs> landmarks) {
  cout<<"[";
  for (unsigned int i = 0; i < landmarks.size(); ++i) {
    cout<<landmarks[i].id;
    if (i < (landmarks.size()-1)) {
      cout<<", ";
    }
  }
  cout<<"]"<<endl;
}

void ParticleFilter::printObs(vector<LandmarkObs> landmarks) {
  cout<<"{";
  for (unsigned int i = 0; i < landmarks.size(); ++i) {
    cout<<"["<<landmarks[i].id<<": ("<<landmarks[i].x<<", "<<landmarks[i].y<<")]";
    if (i < (landmarks.size()-1)) {
      cout<<", ";
    }
  }
  cout<<"}"<<endl;
}