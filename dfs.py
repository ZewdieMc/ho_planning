# Code for the depth first search from the map coordinates --> coordinates with frontier value = 1 is frontier

import numpy as np


class Cell:
    def __init__(self, x, y, is_frontier=False, parent=None, visited=False):
        self.x = 0
        self.y = 0
        self.is_frontier = is_frontier
        self.parent = parent
        self.visited = False


def dfs(frontier_map, start, goal):
    for cell in frontier_map.items():
        cell.visited = True
        


def get_neighbor_cell(map, cell):
    neighbor = []
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
            neighbor.append(map[(nx, ny)])
    return neighbor


def filter_frontier_cells(map):
    frontier_cells = []
    for cell in map.items():  # not sure about item in here
        if cell.is_frontier:
            frontier_cells.append(cell)
    return frontier_cells


example_map = [(), (), (), (), (), (), (), ()]
