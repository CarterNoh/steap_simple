import matplotlib.pyplot as plt
import numpy as np
from gpmp2 import (BodySphere, BodySphereVector, GaussianProcessPriorLinear,
                   ObstaclePlanarSDFFactorGPPointRobot,
                   ObstaclePlanarSDFFactorPointRobot, PlanarSDF, PointRobot,
                   PointRobotModel, interpolateArmTraj)
# from gpmp2.utils.plot_utils import plotSignedDistanceField2D
from gpmp2.utils.signedDistanceField2D import signedDistanceField2D
from gtsam import (DoglegOptimizer, DoglegParams, GaussNewtonOptimizer,
                   GaussNewtonParams, Values, NonlinearFactorGraph, Point2, Point3, Rot2,
                   PriorFactorVector, BearingRangeFactor2D, noiseModel)
from gtsam.symbol_shorthand import V, X, L


class Map():
    def __init__(self, map_size, num_obstacles=30, obstacle_size=4):
        self.height, self.width = map_size
        self.obstacles = self.generateRandomObstacles(num_obstacles, obstacle_size)
        self.map = self.generateMap(self.obstacles, obstacle_size)
        self.sdf = self.createSDF()

    def generateRandomObstacles(self, num_obstacles, obstacle_size):
        obstacles = {}
        for i in range(num_obstacles):
            x = np.random.randint(0, self.width - obstacle_size)
            y = np.random.randint(0, self.height - obstacle_size)
            obstacles[i] = (x,y)
        return obstacles

    def generateMap(self, obs, obs_size):
        map_array = np.zeros((self.height, self.width), dtype=int)
        for i in obs:
            map_array[obs[i][1]:obs[i][1]+obs_size, obs[i][0]:obs[i][0]+obs_size] = 1
        return map_array
    
    def createSDF(self):
        '''Signed Distance Field representation of occupancy map'''
        field = signedDistanceField2D(self.map, 1)
        return PlanarSDF(Point2(0,0), 1, field)

    def visualize(self, ax):
        inv_map_array = 1 - self.map
        ax.imshow(inv_map_array, cmap='gray')


class STEAP():
    def __init__(self):
        # Spacing & Graph Params
        self.n = 5                  # number of nodes
        self.T = 10                 # total time to reach goal
        self.dt = self.T / self.n   # timestep
        self.check_freq = 20        # Number of upsamples between nodes to check for collisions
        self.traj_up = 100          # Number of upsamples between nodes to interpolate GP trajectory
        self.sigma = 0.15           # Smoothness Cost: higher means smoother paths at the expense of obstacle avoidance
        self.eps = 12               # 2 * desired minimum distance to obstacles

        # Trajectory
        self.start_pos = np.array([20, 40])
        self.start_vel = np.array([0, 0])
        self.goal_pos = np.array([180, 60])
        self.goal_vel = np.array([0, 0])
        self.avg_vel = (self.goal_pos - self.start_pos) / self.T
        self.true_traj = []

        # Map
        self.map = Map(map_size=(100, 200), # height, width
                       num_obstacles=15, 
                       obstacle_size=4)

        # Noise Params
        self.vel_noise = 7
        self.meas_noise = [0.1, 0.2]
        self.meas_model = noiseModel.Diagonal.Sigmas(np.array(self.meas_noise))
        self.Qc_model = noiseModel.Gaussian.Covariance(np.identity(2))
        self.goal_fix = noiseModel.Isotropic.Sigma(2, 0.0001)
        

        # Robot Model
        self.robot = self.createRobot()

    def createRobot(self):
        '''Point robot model'''
        pR = PointRobot(2, 1)
        sphere_vec = BodySphereVector()
        sphere_vec.push_back(BodySphere(0, 1.5, Point3(0,0,0)))
        pR_model = PointRobotModel(pR, sphere_vec)
        return pR_model

    def createFactorGraph(self):
        '''
        Builds a factor graph that plans future poses using Gaussian Process estimation 
        acording ot the Gaussian Process Motion Planning algorithm (GPMP2). 
        
        Factors:
        - Nodes: Poses in continuous space at points in continuous time
        - Start & Goal: Unary factors on first and last node to constrain the
          beginning and end of the trajectory
        - GP Prior: Binary factors between nodes that represent continuous-time 
          trajectory function
        - Obstacle Cost: Unary factor at each node representing distance to nearest obstacle 
        - Obstacle GP Cost: Binary factor between nodes representing an estimate of the 
          full trajectory's distance from obstacles
        - Measurement: Added during STEAP algorithm execution. Unary factor representing 
          a measurement of position. 

        Solving this factor graph as built is an implementation of the Gaussian Process 
        Motion Planning algorithm (GPMP2). 
        '''
        
        self.graph = NonlinearFactorGraph()
        init_values = Values()

        # Initialize Nodes & Factors
        for i in range(0, self.n + 1):
            key_pos = X(i)
            key_vel = V(i)

            # Initialize trajectory as straight line
            pose = self.start_pos * float(self.n - i) / float(self.n) + \
                   self.goal_pos * i / float(self.n)
            vel = self.avg_vel
            init_values.insert(key_pos, pose)
            init_values.insert(key_vel, vel)

            # Start and end prior factors
            if i == 0:
                self.graph.add(PriorFactorVector(key_pos, self.start_pos, self.goal_fix))
                self.graph.add(PriorFactorVector(key_vel, self.start_vel, self.goal_fix))
            elif i == self.n:
                self.graph.add(PriorFactorVector(key_pos, self.goal_pos, self.goal_fix))
                self.graph.add(PriorFactorVector(key_vel, self.goal_vel, self.goal_fix))

            # Initial measurement factor
            if i == 0:
                est_pos, cov = self.getMeasurement(pose)
                self.graph.add(PriorFactorVector(X(0), est_pos, noiseModel.Gaussian.Covariance(cov)))

            # GP motion prior factors and obstacle cost factors
            if i > 0:
                key_pos1 = X(i-1)
                key_pos2 = X(i)
                key_vel1 = V(i-1)
                key_vel2 = V(i)

                # GP motion prior
                gp_prior = GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2,
                                                key_vel2, self.dt, self.Qc_model)
                self.graph.add(gp_prior)

                # Obstacle cost factor (unary)
                obs_cost = ObstaclePlanarSDFFactorPointRobot(key_pos, self.robot, self.map.sdf,
                                                    self.sigma, self.eps)
                self.graph.add(obs_cost)

                # GP obstacle cost factor (binary)
                if self.check_freq > 1:
                    for j in range(1, self.check_freq+1):
                        tau = j * (1/self.check_freq)
                        obs_cost_gp = ObstaclePlanarSDFFactorGPPointRobot(key_pos1, key_vel1, key_pos2, key_vel2,
                                self.robot, self.map.sdf, self.sigma, self.eps, self.Qc_model, self.dt, tau)
                        self.graph.add(obs_cost_gp)
                            
        parameters = GaussNewtonParams()
        self.optimizer = GaussNewtonOptimizer(self.graph, init_values, parameters)

        print(f"Initial Error: {self.graph.error(init_values)}\n")

    def interpolateGP(self, result, i):
        traj = interpolateArmTraj(result, self.Qc_model, self.dt, self.traj_up, i, i+1)
        pos_list = []
        vel_list = []
        for i in range(self.traj_up):
            pos_list.append(traj.atVector(X(i)))
            vel_list.append(traj.atVector(V(i)))
        return (pos_list, vel_list)
        
    def isCollision(self, traj):
        # Check each location for collisions using SDF
        for pos in traj:
            if self.map.sdf.getSignedDistance(Point2(pos)) < 0:
                return True
        return False

    def execute(self, traj, p0):
        vel_list = traj[1]
        dt = self.dt / self.traj_up
        p = p0
        for vel in vel_list:
            p_next = p + dt * (vel + np.random.normal(0, self.vel_noise, 2))
            self.true_traj.append(p_next)
            p = p_next
        if self.isCollision(self.true_traj):
            raise ValueError("Robot encountered collision on trajectory")
        return p

    def getMeasurement(self, pos):
        # Random obstacle
        obst_idx = np.random.randint(len(self.map.obstacles))
        obst = self.map.obstacles[obst_idx]

        # Noisy range and bearing measurement to obstacle (x,y) position
        vec = np.array(obst) - np.array(pos)
        r = np.linalg.norm(vec) + np.random.normal(0, self.meas_noise[0])
        b = np.arctan2(vec[1], vec[0]) + np.random.normal(0, self.meas_noise[1])

        # From measurements and known landmark, back out estimate of position
        est_pos = np.array(obst) - r * np.array([np.cos(b), np.sin(b)])

        # propagate uncertainty
        E_rb = np.diag(self.meas_noise)
        J = np.array([[1, -np.sin(b)],[1, np.cos(b)]])
        E_xy = J @ E_rb @ J.T
        return est_pos, E_xy

    def plotGraph(self, result, idx):
        '''Plot Graph at a given timestep'''
        # Extract trajectory
        plot_steps = self.n * (self.traj_up + 1)
        plot_values = interpolateArmTraj(result, self.Qc_model, self.dt, self.traj_up)
        positions = np.zeros((2, plot_steps))
        for i in range(plot_steps):
            pos = plot_values.atVector(X(i))
            positions[0,i] = pos[0]
            positions[1,i] = pos[1]

        # Extract true trajectory
        true = np.array(self.true_traj).T

        # Plot
        figure = plt.figure()
        axis = figure.gca()
        self.map.visualize(axis)
        k = idx * self.traj_up
        axis.plot(positions[0][:k+1], positions[1][:k+1], 'b-') # estimated trajectory
        axis.plot(positions[0][k:], positions[1][k:], 'r-') # planned trajectory
        axis.plot(true[0], true[1], 'g-') # true trajectory
        axis.plot(positions[0][k], positions[1][k], 'g.', markersize=10) # current position
        axis.plot(self.start_pos[0], self.start_pos[1], 'k.', markersize=10) # goal position
        axis.plot(self.goal_pos[0], self.goal_pos[1], 'k.', markersize=10)
        axis.legend(("Estimated", "Planned", "True"))
        axis.set_title(f"Step {idx}")
        
        plt.savefig(f"output/{idx}.png", dpi=300)

    def solveGraph(self, i):
        # GP is stochastic, so as a heuristic, solve 5 times and pick lowest cost
        best_error = np.inf
        best_result = None
        for i in range(10):
            self.optimizer.optimizeSafely()
            result = self.optimizer.values()
            error = self.graph.error(result)
            if error < best_error:
                best_error = error
                best_result = result
        return best_result

    def simulate(self):
        # Initialize trajectory and factor graph and plan first trajectory
        self.createFactorGraph()
        result = self.solveGraph(0)
        print(f"Error at 0: {self.graph.error(result)}\n")

        p = result.atVector(X(0))
        self.true_traj.append(p)
        self.plotGraph(result, 0)
        
        for i in range(self.n):

            # Interpolate GP to get upsampled future trajectory to next node
            traj = self.interpolateGP(result, i)
            iter = 0
            while self.isCollision(traj[0]) or iter < 10: # if collision, replan
                traj = self.interpolateGP(result, i)
                iter += 1
            if self.isCollision(traj[0]):
                raise ValueError("Could not plan trajectory without collisions.")

            # Travel trajectory
            pos = self.execute(traj, p) 
            p = pos

            # Get measurement & update graph
            est_pos, cov = self.getMeasurement(pos)
            self.graph.add(PriorFactorVector(X(i+1), est_pos, noiseModel.Gaussian.Covariance(cov)))
            
            # Resolve trajectory
            self.optimizer.optimizeSafely()
            self.result = self.optimizer.values()
            print(f"Error at {i+1}: {self.graph.error(self.result)}\n")

            self.plotGraph(result, i+1)
                



if __name__ == "__main__":
    steap = STEAP()
    steap.simulate()
    # print("Factor Graph:\n{}".format(steap.graph))
    
    