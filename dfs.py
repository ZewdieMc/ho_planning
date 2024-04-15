# Code for the depth first search from the map coordinates --> coordinates with frontier value = 1 is frontier

import numpy as np


class Cell:
    def __init__(self, x, y, is_frontier=False, parent=None, visited=False):
        self.x = 0
        self.y = 0
        self.is_frontier = is_frontier
        self.parent = parent
        self.visited = False


def dfs(map):
    frontier_cells = filter_frontier_cells(map)
    visited_cells = set()
    clusters = []

    for cell in frontier_cells:
        if cell not in visited_cells:
            cluster = set()
            explore_cluster(map, cell, cluster, visited_cells)
            clusters.append(cluster)

    return clusters

def explore_cluster(map, cell, cluster, visited_cells):
    visited_cells.add(cell)
    cluster.add(cell)

    neighbor_cells = get_neighbor_cells(map, cell)
    for neighbor in neighbor_cells:
        if neighbor not in visited_cells and neighbor.is_frontier:
            explore_cluster(map, neighbor, cluster, visited_cells)


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

        # check if frontier or in map
        if (nx, ny) in map and map[(nx, ny)].is_frontier:
            neighbors.append(map[(nx, ny)])
    return neighbors


def filter_frontier_cells(map):
    frontier_cells = []
    for cell in map.items():  # not sure about item in here
        if cell.is_frontier:
            frontier_cells.append(cell)
    return frontier_cells

#Test in here
example_map = [(), (), (), (), (), (), (), ()]
