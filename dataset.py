import numpy as np
import matplotlib.pyplot as plt

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

# Example usage
if __name__ == "__main__":
    map_size = (200, 100)  # width, height
    num_obstacles = 30
    obstacle_size = 5

    map_array = create_random_map(map_size, num_obstacles, obstacle_size)
    visualize_map(map_array)