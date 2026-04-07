import matplotlib.pyplot as plt
import numpy as np
from gpmp2 import (BodySphere, BodySphereVector, GaussianProcessPriorLinear,
                   ObstaclePlanarSDFFactorGPPointRobot,
                   ObstaclePlanarSDFFactorPointRobot, PlanarSDF, PointRobot,
                   PointRobotModel)
from gpmp2.datasets.generate2Ddataset import generate2Ddataset
from gpmp2.utils.plot_utils import (plotEvidenceMap2D, plotPointRobot2D,
                                    plotSignedDistanceField2D)
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

    def visualize(self):
        inv_map_array = 1 - self.map
        extent = (-self.origin[0]*cell_size, (self.width-self.origin[0])*cell_size, 
                  -self.origin[1]*cell_size, (self.height-self.origin[1])*cell_size)
        plt.imshow(inv_map_array, cmap='gray', extent=extent) # 
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()


class STEAP():
    def __init__(self, n, T, start, goal, map, num_checks, Qc=np.identity(2), cost_sigma=0.5, eps_dist=4, 
                  goal_cov=(0.0001, 0.0001), use_trustregion_opt=False):
        # Spacing & Graph Params
        self.n = n
        self.T = T
        self.dt = T / n
        self.n_checks = num_checks
        self.check_interval = int(num_checks / num_nodes - 1)

        # Start/Goal
        self.start_pos = np.array(start[0:2])
        self.start_vel = np.array(start[2:4])
        self.goal_pos = np.array(goal[0:2])
        self.goal_vel = np.array(goal[2:4])
        self.avg_vel = (self.goal_pos - self.start_pos) / n / self.dt

        # Map
        self.map = map
        self.sdf = self.createSDF()

        # Noise Params
        self.Qc_model = noiseModel.Gaussian.Covariance(Qc)
        self.sigma = cost_sigma
        self.eps = eps_dist
        self.pose_fix = noiseModel.Isotropic.Sigma(2, goal_cov[0])
        self.vel_fix = noiseModel.Isotropic.Sigma(2, goal_cov[1])
        self.tr_opt = use_trustregion_opt

        # Robot Model
        self.robot = self.createRobot()
        

    def createSDF(self):
        # Signed Distance Field representation of occupancy map
        origin_point2 = Point2(self.map.origin[0], self.map.origin[1])
        field = signedDistanceField2D(self.map.map, self.map.cell_size)
        sdf = PlanarSDF(origin_point2, self.map.cell_size, field)

        # plot
        figure1 = plt.figure(0)
        axis1 = figure1.gca()
        plotSignedDistanceField2D(figure1, axis1, field, 
                                  self.map.origin[0], self.map.origin[1], self.map.cell_size)
        return sdf

    def createRobot(self):
        # point robot model
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
        
        graph = NonlinearFactorGraph()
        init_values = Values()

        for i in range(0, self.n + 1):
            key_pos = X(i)
            key_vel = V(i)

            # Initialize trajectory as straight line
            pose = self.start_pos * float(self.n - i) / float(
                self.n) + self.goal_pos * i / float(self.n)
            vel = self.avg_vel
            print(pose)
            init_values.insert(key_pos, pose)
            init_values.insert(key_vel, vel)

            # Start and end prior factors
            if i == 0:
                graph.push_back(PriorFactorVector(key_pos, self.start_pos, self.pose_fix))
                graph.push_back(PriorFactorVector(key_vel, self.start_vel, self.vel_fix))
            elif i == self.n:
                graph.push_back(PriorFactorVector(key_pos, self.goal_pos, self.pose_fix))
                graph.push_back(PriorFactorVector(key_vel, self.goal_vel, self.vel_fix))

            # GP motion prior factors and obstacle cost factors
            if i > 0:
                key_pos1 = X(i-1)
                key_pos2 = X(i)
                key_vel1 = V(i-1)
                key_vel2 = V(i)

                # GP motion prior
                gp_prior = GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2,
                                                key_vel2, self.dt, self.Qc_model)
                graph.push_back(gp_prior)

                # Obstacle cost factor (unary)
                graph.push_back(
                    ObstaclePlanarSDFFactorPointRobot(key_pos, self.robot, self.sdf,
                                                    self.sigma, self.eps))

                # GP obstacle cost factor (binary)
                if self.check_interval > 0:
                    for j in range(1, self.check_interval + 1):
                        tau = j * (self.n / self.n_checks)
                        graph.add(
                            ObstaclePlanarSDFFactorGPPointRobot(key_pos1, key_vel1, key_pos2, key_vel2,
                                self.robot, self.sdf, self.sigma, self.eps, self.Qc_model, self.dt, tau))
        
        if self.tr_opt:
            parameters = DoglegParams()
            parameters.setVerbosity("ERROR")
            optimizer = DoglegOptimizer(graph, init_values, parameters)
        else:
            parameters = GaussNewtonParams()
            # parameters.setRelativeErrorTol(1e-5)
            # parameters.setMaxIterations(100)
            # parameters.setVerbosity("ERROR")
            optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

        print(f"Initial Error = {graph.error(init_values)}\n")
        optimizer.optimizeSafely()
        result = optimizer.values()
        print(f"Final Error = {graph.error(result)}\n")

        # plot final values
        figure = plt.figure(1)
        axis = figure.gca()
        # plot world
        plotEvidenceMap2D(figure, axis, self.map.map, origin[0], origin[1], cell_size)
        for i in range(int(self.T) + 1):
            axis.set_title("Optimized Values")
            # plot arm
            pos = result.atVector(X(i))
            plotPointRobot2D(figure, axis, self.robot, pos)
            plt.pause(self.n / self.n_checks)

        return graph, optimizer

    def addMeasurementFactor(FG, idx):
        # Returns the new FG
        pass

    def inferFactorGraph(FG):
        # returns trajectory Theta
        pass


if __name__ == "__main__":

    #### Project Settings ####

    # Map Settings
    map_size = (200, 100)
    origin = (0, 0)
    cell_size = 1
    num_obstacles = 30
    obstacle_size = 5
    
    # STEAP Settings
    num_nodes = 20
    total_time = 10.0 # sec
    num_checks = 50.0
    start = [0, 0, 0, 0] # pos_x, pos_y, vel_x, vel_y
    goal = [17, 14, 0, 0] # pos_x, pos_y, vel_x, vel_y

    # Plotting Settings
    pause_time = num_nodes / total_time # for plotting


    #### Simulation ####
    sim_map = Map(map_size, origin, cell_size, num_obstacles, obstacle_size)
    steap = STEAP(num_nodes, total_time, start, goal, sim_map, num_checks)

    steap.createFactorGraph()
    