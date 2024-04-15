# Code for the depth first search from the map coordinates --> coordinates with frontier value = 1 is frontier

import numpy as np
from typing import Dict, List
import matplotlib.pyplot as plt

class Cell:
    def __init__(self, x = 0, y = 0, is_frontier=False, parent=None, visited=False):
        self.x = x
        self.y = y
        self.is_frontier = is_frontier
        self.parent = parent
        self.visited = False


def get_clusters(map):
    clusters = []
    visited = set()

    for cell in map:
        if map[cell].is_frontier and cell not in visited:
            cluster = []
            stack = [cell]

            while stack:
                current = stack.pop()
                if current not in cluster and map[current].is_frontier:
                    cluster.append(current)
                    for neighbor in get_neighbor_cells(map, current):
                        if map[neighbor].is_frontier:
                            stack.append(neighbor)

            clusters.append(cluster)
            visited.update(cluster)

    return clusters



def get_neighbor_cells(map, cell):
    neighbors = []
    x, y = cell.x, cell.y
    for delta_x, delta_y in [
        (-1, -1),
        (0, -1),
        (1, -1),
        (1, 0),
        (1, 1),
        (0, 1),
        (-1, 1),
        (-1, 0),
    ]:
        # get neighbor cell
        nx, ny = x + delta_x, y + delta_y
        if 0 <= nx < len(map) and 0 <= ny < len(map[0]):
            neighbors.append(map[nx][ny]) # in here, I want to be able to add a Cell object, not just a set of coordinates. How?
    return neighbors

#passes the test cases
def filter_frontier_cells(map):
    frontier_cells = []
    for cell in map.values():  # not sure 
        if cell.is_frontier:
            frontier_cells.append(cell)
    return frontier_cells

#Test in here

map = {
    (0,0): Cell(x=0, y=0, is_frontier=False),
    (0,1): Cell(x=0, y=1, is_frontier=False),
    (0,2): Cell(x=0, y=2, is_frontier=True),
    (1,0): Cell(x=1, y=0, is_frontier=False),
    (1,1): Cell(x=1, y=1, is_frontier=True),
    (1,2): Cell(x=1, y=2, is_frontier=False),
    (2,0): Cell(x=2, y=0, is_frontier=False),
    (2,1): Cell(x=2, y=1, is_frontier=True),
    (2,2): Cell(x=2, y=2, is_frontier=False)
    }

# This is just for visualization, not necessary for the algorithm
# def visualize_clusters(map, clusters):
#     map_visual = [[0 for _ in range(len(map[0]))] for _ in range(len(map))]

#     for color, cluster in enumerate(clusters, 1):
#         for cell in cluster:
#             map_visual[cell.x][cell.y] = color

#     plt.imshow(map_visual, cmap='nipy_spectral')
#     plt.show()
clusters = get_clusters(map)
print(clusters)



