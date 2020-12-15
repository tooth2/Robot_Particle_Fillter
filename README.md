# Overview
This repository contains all the code needed to accomplish Localization -by implementing a 2D particle filter in C++.

## Project Overview
A robot has been transported to a new location. It has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. The robot should be localized by applying Particle filter. (Particle filter is often used  to solve  localization problem). In this project the particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the particle filter will also get observation and control data.

#### Code structure
The directory structure of this repository is as follows:

```
root
|___
|   data
|   |   map_data.txt
|___src
|   helper_functions.h
|   main.cpp
|   map.h
|   particle_filter.cpp
|   particle_filter.h
```
 * `src/main.cpp` :  This file contains the code that will actually be running a particle filter and calling the associated methods. There are some parameters in which govern the requirements on accuracy and run time.
 *  `src/particle_filter.cpp` :The file contains the scaffolding of a `ParticleFilter` class and some associated methods.
 * `src/particle_filter.h`: the header file of `ParticleFilter`

#### main.cpp

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

```
INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator
["sense_x"] ["sense_y"] ["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state
["previous_velocity"] ["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
["sense_observations_x"] ["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation
["best_particle_x"] ["best_particle_y"] ["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations
// for respective (x,y) sensed positions ID label
["best_particle_associations"]

// for respective (x,y) sensed positions
["best_particle_sense_x"] <= list of sensed x positions
["best_particle_sense_y"] <= list of sensed y positions

```

## Implementing the Particle Filter
### **initialization** *ParticleFilter::init*
At the initialization step, this estimates the position from GPS input. The subsequent steps in the process will refine this estimate to localize the vehicle. As with all sensor based operations, this step is impacted by sensor noise. So Gaussian distribution is applied: normal distribution with mean equal to the GPS position and standard deviation for the x and y position.
  * each of particle position is generated randomply and set the wieght =1
  * Set the number of particles: Number of particles is set 100. (1000 works fine too)
  * Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties std[] from GPS)
  * set all weights to 1.
  * Add Gaussian noise to each particle: `std::default_random_engine gen` is the random engine initialized earlier.
   ``` code
  //normal (Gaussian) noise distribution for x, y, theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
 ```
 
 ### **Prediction** *ParticleFilter::prediction*
After initializing all particles it's time to predict the vehicle's position, where the vehicle will be at the next time step, by updating based on yaw rate and velocity, while accounting for Gaussian sensor noise.
During the prediction step , the control input (yaw rate & velocity) for all particles  is added
 Sensor noise: represent uncertaintly : respond particles with random gausian
 * Add measurements to each particle
 * add random Gaussian noise.
 ```code
 // add a random normal (Gaussian) noise distribution for x, y, theta to each measurement
 std::normal_distribution<double> dist_x(new_x, std_pos[0]);
 std::normal_distribution<double> dist_y(new_y, std_pos[1]);
 std::normal_distribution<double> dist_theta(new_theta, std_pos[2]);
 ```
 * When adding noise, std::normal_distribution and std::default_random_engine are used.
When yaw_rate is too small close to 0:
 ```code
new_x = p.x+velocity*delta_t*cos(p.theta);
new_y = p.y+velocity*delta_t*sin(p.theta);
 ```
 
 ### **Nearest Neighbor Map Landmark** *ParticleFilter::dataAssociation*
Measurement landmark associations: Each measurement will need to be associated with a landmark identifier, take the closest landmark to each transformed observation
Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.

### **Assign Association** *ParticleFilter::SetAssociations*
particle: the particle to which assign each listed association, and association's (x,y) world coordinates mapping
associations: The landmark id that goes along with each listed association
sense_x: the associations x mapping already converted to world coordinates
sense_y: the associations y mapping already converted to world coordinates
 
### **Update Step(Update Weights)** *ParticleFilter::updateWeights*
-  transform the car's measurements from its local car coordinate system to the map's coordinate system.
-  to calculate the weight value of the particle.
- This updates particle weights using map landmark positions and measurements.
Note that the x and y errors are depicted from the point of view of the map (x is horizontal, y is vertical) rather than the point of view of the car where x is in the direction of the car’s heading,( i.e. It points to where the car is facing), and y is orthogonal (90 degrees) to the left of the x-axis (pointing out of the left side of the car). Now that we have incorporated velocity and yaw rate measurement inputs into the Particle filter, this step updates particle weights based on LIDAR and RADAR readings of landmarks.

 The particle's final weight will be calculated as the product of each measurement's Multivariate-Gaussian probability density.
 The Multivariate-Gaussian probability density has two dimensions, x and y. The mean of the Multivariate-Gaussian is the measurement's associated landmark position and the Multivariate-Gaussian's standard deviation is described by initial uncertainty in the x and y ranges. The Multivariate-Gaussian is evaluated at the point of the transformed measurement's position.

The observations are given in the vehicle's coordinate system. However, particles are located according to the MAP'S coordinate system. So these coords should be transformed between the two systems for both rotation and translation.

### **Resample** *ParticleFilter::resampl*
Resample particles with replacement with probability proportional to their weight.
 ```code
discrete_distribution<int> distribution(weights.begin(), weights.end());
 ```
 
### Inputs to the Particle Filter
inputs to the particle filter -->  `data` directory.

#### The Map*<vector>
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position : Landmark x-position in the map (global coordinates)
2. y position : Landmark y-position in the map (global coordinates)
3. landmark id : Landmark ID

#### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

#### Control data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent

1. velocity [m/s] : vehicle speed (in meters per second)
2. yawrate [rad/s] : vehicle yaw rate (in radians per second)

#### Observation data(noisy)
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place.

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:

x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE.
y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.
NOTE The vehicle's coordinate system is NOT the map coordinate system. The code will handle this transformation.
distance: Computes the Euclidean distance between two 2D points.
e.g )  sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
#### Exit Criteria:
When  the particle filter localizes the vehicle to within the desired accuracy and performance  while running `.\run.sh`,  update prediction to avoid yaw_rate is too small (the yaw_rate <0.00001)

1. **Accuracy**: The particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: The particle filter should complete execution within the time of 100 seconds.

## Running the Code
This project requires [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/) that run c++ uWebSocketIO, which is used to communicate with the simulator. The simulator provides the script for the noisy position data, vehicle controls, and noisy observations. The script feeds back the best particle state. This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windowseither Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO can be used.
The simulator can also display the best particle's sensed positions, along with the corresponding map ID associations. This can be extremely helpful to make sure transition and association calculations were done correctly. When the simulator the green laser sensors from the car nearly overlap the blue laser sensors from the particle, this means that the particle transition calculations were done correctly.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory. Executable unit environment is ./particle_filter

