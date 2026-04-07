import matplotlib.pyplot as plt
import numpy as np
from gpmp2 import (BodySphere, BodySphereVector, GaussianProcessPriorLinear,
                   ObstaclePlanarSDFFactorGPPointRobot,
                   ObstaclePlanarSDFFactorPointRobot, PlanarSDF, PointRobot,
                   PointRobotModel)
# from gpmp2.utils.plot_utils import plotSignedDistanceField2D
from gpmp2.utils.signedDistanceField2D import signedDistanceField2D
from gtsam import (DoglegOptimizer, DoglegParams, GaussNewtonOptimizer,
                   GaussNewtonParams, NonlinearFactorGraph, Point2, Point3,
                   PriorFactorVector, Values, noiseModel)
from gtsam.symbol_shorthand import V, X


class Map():
    def __init__(self, map_size, origin=(0,0), cell_size=1, num_obstacles=30, obstacle_size=5):
        self.height, self.width = map_size
        self.origin = origin
        self.cell_size = cell_size
        self.obstacles = self.generate_random_obstacles(num_obstacles, obstacle_size)
        self.map = self.generate_map(self.obstacles, obstacle_size)

    def generate_random_obstacles(self, num_obstacles, obstacle_size):
        obstacles = {}
        for i in range(num_obstacles):
            x = np.random.randint(0, self.width - obstacle_size)
            y = np.random.randint(0, self.height - obstacle_size)
            obstacles[i] = (x,y)
        return obstacles

    def generate_map(self, obs, obs_size):
        map_array = np.zeros((self.height, self.width), dtype=int)
        for i in obs:
            map_array[obs[i][1]:obs[i][1]+obs_size, obs[i][0]:obs[i][0]+obs_size] = 1
        return map_array

    def visualize(self, ax):
        inv_map_array = 1 - self.map
        extent = (-self.origin[0]*cell_size, (self.width-self.origin[0])*cell_size, 
                  -self.origin[1]*cell_size, (self.height-self.origin[1])*cell_size)
        ax.imshow(inv_map_array, cmap='gray', extent=extent) # 
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')


class STEAP():
    def __init__(self, num_nodes, T, start, goal, map, collision_check_freq, Qc=1, 
                 cost_sigma=0.1, eps_dist=4, goal_cov=(0.0001, 0.0001), trustregion_opt=True):
        # Spacing & Graph Params
        self.n = num_nodes
        self.T = int(T)
        self.dt = T / num_nodes
        self.check_freq = collision_check_freq

        # Start/Goal
        self.start_pos = np.array(start[0:2])
        self.start_vel = np.array(start[2:4])
        self.goal_pos = np.array(goal[0:2])
        self.goal_vel = np.array(goal[2:4])
        self.avg_vel = (self.goal_pos - self.start_pos) / T

        # Map
        self.map = map
        self.sdf = self.createSDF()

        # Noise Params
        self.Qc_model = noiseModel.Gaussian.Covariance(np.identity(2)*Qc)
        self.sigma = cost_sigma
        self.eps = eps_dist
        self.pose_fix = noiseModel.Isotropic.Sigma(2, goal_cov[0])
        self.vel_fix = noiseModel.Isotropic.Sigma(2, goal_cov[1])
        self.tr_opt = trustregion_opt

        # Robot Model
        self.robot = self.createRobot()
        
    def createSDF(self):
        '''Signed Distance Field representation of occupancy map'''
        origin_point2 = Point2(self.map.origin[0], self.map.origin[1])
        field = signedDistanceField2D(self.map.map, self.map.cell_size)
        return PlanarSDF(origin_point2, self.map.cell_size, field)
    


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
        
        Structure of factor graph:
        - Nodes: Poses in continuous space at points in continuous time
        - Start & Goal: Unary factors on first and last node to constrain the
          beginning and end of the trajectory
        - GP Prior: Binary factors between nodes that represent continuous-time 
          trajectory function
        - Obstacle Cost: Unary factor at each node representing distance to nearest obstacle 
        - Obstacle GP Cost: Binary factor between nodes representing an estimate of the 
          full trajectory's distance from obstacles

        Solving this factor graph as built is an implementation of the Gaussian Process 
        Motion Planning algorithm (GPMP2). 
        '''
        
        self.graph = NonlinearFactorGraph()
        init_values = Values()

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
                self.graph.push_back(PriorFactorVector(key_pos, self.start_pos, self.pose_fix))
                self.graph.push_back(PriorFactorVector(key_vel, self.start_vel, self.vel_fix))
            elif i == self.n:
                self.graph.push_back(PriorFactorVector(key_pos, self.goal_pos, self.pose_fix))
                self.graph.push_back(PriorFactorVector(key_vel, self.goal_vel, self.vel_fix))

            # GP motion prior factors and obstacle cost factors
            if i > 0:
                key_pos1 = X(i-1)
                key_pos2 = X(i)
                key_vel1 = V(i-1)
                key_vel2 = V(i)

                # GP motion prior
                gp_prior = GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2,
                                                key_vel2, self.dt, self.Qc_model)
                self.graph.push_back(gp_prior)

                # Obstacle cost factor (unary)
                obs_cost = ObstaclePlanarSDFFactorPointRobot(key_pos, self.robot, self.sdf,
                                                    self.sigma, self.eps)
                self.graph.push_back(obs_cost)

                # GP obstacle cost factor (binary)
                if self.check_freq > 1:
                    for j in range(1, self.check_freq+1):
                        tau = j * (1/self.check_freq)
                        obs_cost_gp = ObstaclePlanarSDFFactorGPPointRobot(key_pos1, key_vel1, key_pos2, key_vel2,
                                self.robot, self.sdf, self.sigma, self.eps, self.Qc_model, self.dt, tau)
                        self.graph.add(obs_cost_gp)
                            
        
        if self.tr_opt:
            parameters = DoglegParams()
            # parameters.setVerbosity("ERROR")
            self.optimizer = DoglegOptimizer(self.graph, init_values, parameters)
        else:
            parameters = GaussNewtonParams()
            # parameters.setRelativeErrorTol(1e-5)
            # parameters.setMaxIterations(100)
            # parameters.setVerbosity("ERROR")
            self.optimizer = GaussNewtonOptimizer(self.graph, init_values, parameters)

        print(f"Initial Error = {self.graph.error(init_values)}\n")
        self.optimizer.optimizeSafely()
        result = self.optimizer.values()
        print(f"Final Error = {self.graph.error(result)}\n")

        return result

    def plot_graph(self, result, idx, title):
        '''Plot Graph at a given timestep'''
        # Extract node positions
        positions = np.zeros((2,self.n))
        for i in range(self.n):
            pos = result.atVector(X(i))
            positions[0,i] = pos[0]
            positions[1,i] = pos[1]
        # Plot
        figure = plt.figure()
        axis = figure.gca()
        self.map.visualize(axis)
        axis.plot(positions[0][:idx+1], positions[1][:idx+1], 'b-')
        axis.plot(positions[0][idx:], positions[1][idx:], 'r-')
        axis.plot(positions[0][idx], positions[1][idx], 'g.', markersize=10)
        axis.plot(self.start_pos[0], self.start_pos[1], 'k.', markersize=10)
        axis.plot(self.goal_pos[0], self.goal_pos[1], 'k.', markersize=10)
        
        axis.set_title(title)
        plt.savefig(f"output/{title}.png", dpi=300)

    def addMeasurementFactor(FG, idx):
        # Returns the new FG
        pass

    def inferFactorGraph(FG):
        # returns trajectory Theta
        pass


if __name__ == "__main__":

    #### Project Settings ####
    # Map Settings
    map_size = (100, 200) # height, width
    origin = (0, 0)
    cell_size = 1
    num_obstacles = 30
    obstacle_size = 5
    # STEAP Settings
    num_nodes = 20
    total_time = 10.0 # sec
    collision_check_freq = 5 # Per node
    start= [20, 40, 0, 0] # pos_x, pos_y, vel_x, vel_y
    goal = [180, 60, 0, 0] # pos_x, pos_y, vel_x, vel_y

    #### Simulation ####
    sim_map = Map(map_size, origin, cell_size, num_obstacles, obstacle_size)
    steap = STEAP(num_nodes, total_time, start, goal, sim_map, collision_check_freq)

    result = steap.createFactorGraph()
    steap.plot_graph(result, 10, "test")
    