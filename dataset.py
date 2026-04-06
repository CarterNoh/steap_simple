import numpy as np
import matplotlib.pyplot as plt
import gtsam
from gtsam import (DoglegOptimizer, DoglegParams, GaussNewtonOptimizer,
                   GaussNewtonParams, NonlinearFactorGraph, Point2, Point3,
                   PriorFactorVector, Values, noiseModel)
from gtsam.symbol_shorthand import V, X

def generate_random_obstacles(map_size, num_obstacles, obstacle_size):
    pass #return list or dict of obstacles/landmarks

def create_random_map(map_size, num_obstacles, obstacle_size):
    width, height = map_size
    map_array = np.zeros((height, width), dtype=int)
    for _ in range(num_obstacles):
        x = np.random.randint(0, width - obstacle_size)
        y = np.random.randint(0, height - obstacle_size)
        map_array[y:y+obstacle_size, x:x+obstacle_size] = 1
    return map_array

def visualize_map(map_array):
    inv_map_array = 1 - map_array
    plt.imshow(inv_map_array, cmap='gray')
    # plt.title('Map with Obstacles')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def generate_sdf(map, cell_size, origin):
    pass

def crateFactorGraph():
    pass

def addMeasurementFactor(FG, idx):
    # Returns the new FG
    pass

def inferFactorGraph(FG):
    # returns trajectory Theta
    pass




# Example usage
if __name__ == "__main__":
    map_size = (200, 100)  # width, height
    num_obstacles = 30
    obstacle_size = 5

    map = create_random_map(map_size, num_obstacles, obstacle_size)
    # visualize_map(map)
    


