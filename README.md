# STEAP: Simultaneous Trajectory Estimation and Mapping
This is an implementation of the Simultaneous Trajectory Estimation and Mapping (STEAP) algorithm for a simple 2D planar point robot poblem. STEAP uses a Gaussian Process (GP) to represent continuous-time trajectories through a continuous state space. The GP is incorporated into a Factor Graph to accomplish both trajectory estimation (a problem looking backwards in time from the robot's current state) and trajectory planning (a problem looking forward in time). In STEAM, trajectory estimation and motion planning are seen as two parts of the same process of estimating a continuous-time trajectory. 

STEAM and its predecessor GPMP2 fram the problem of motion planning as probabalistic inference on a markov network. This is somewhat different from the familiar approach of MDPs. Instead of trying to find a universal policy for all areas of the state space, the optimal trajectory from the current state is found by finding the maximum a posteriori estimate of a sequence of states (in this case, by optimizing those parameters through the factor graph framework). This can be seen as a form of Model Predictive Control. 

This algorithm was programmed using the GTSAM library, the industry standard library for factor graphs. It also uses the GPMP2 library, which build on GTSAM to add GP and planning-related functionality. The code is written in Python using the provided Python wrappers of the C++ code. The two libraries must be installed and built to use. To simpleify this, I am using Docker to provide a pre-made virtual environment with all necessary libraries and prerequisites. Building the image and running the container will execute the python script that implements the simulation. 


## Instructions
0. Clone repository
2. Create "output" folder in repository directory
2. Build Docker image from the included dockerfile
3. Run docker image using "docker run --rm -v $(pwd):/steap -v $(pwd)/output:/steap/output steapsimple:latest"
   (or for windows: "docker run --rm -v ${PWD}:/steap -v ${PWD}/output:/steap/output steapsimple:latest")


## Notes
The code has two bugs that occasionally throw exceptions. If either of these happens, simply rerun the program to generate a new random map and trajectory.
   - The planning algorithm occasionally plans trajectories outside of the bounds of the map, triggering an exception from the SDF function. I did not have time to create a workaround to catch this exception.
   - The planner still occasionally plans a trajectory that collides with an obstacle. This could be solved by tuning the "sigma" and "epsilon" parameters in the STEAP class. Sigma controls smoothness, with higher values resulting in smoother paths at the cost of worse obstacle avoidance. Epsilon is 2x the minimum desired distance from obstacles. I did my best to tune these but did not get a perfect result. As a result, sometimes the robot will collide with an obstacle, which will throw an exception. 


## Citations
1. Mukadam, M., Dong, J., Dellaert, F. et al. STEAP: simultaneous trajectory estimation and planning. Auton Robot 43, 415–434 (2019). https://doi.org/10.1007/s10514-018-9770-1
2. Mukadam, M., Dong, J., Yan, X. et al. Continuous-time Gaussian Process Motion Planning via Probabilistic Inference. International Journal of Robotics Research, 2018. https://doi.org/10.1177/0278364918790369
3. Frank Dellaert and GTSAM Contributors. borglab/gtsam. May 2022, Georgia Tech Borg Lab. https://doi.org/10.5281/zenodo.5794541, https://github.com/borglab/gtsa

