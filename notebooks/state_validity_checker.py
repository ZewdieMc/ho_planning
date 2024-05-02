import numpy as np
import math

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data # np.array already
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.bound = [resolution*data.shape[0] ]

    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 
        if self.there_is_map == False:
            return(False)
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        for x in np.arange(pose[0]-self.distance,pose[0]+self.distance,self.resolution):
            for y in np.arange(pose[1]-self.distance,pose[1]+self.distance,self.resolution):
                m = self.__position_to_map__(np.array([x,y]))

        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.

                if m is None:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==-1:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==100:
                    return(False)
                    
        return(True)

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        step_size = 2*self.distance
        for i in range(len(path)-1):
            dist = np.linalg.norm(path[i] - path[i+1])
            zeta = np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])

            step = int(dist//step_size)

        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.
        # For each point between 2 waypoints check if `is_valid`. 
            for j in range(step+1):
                inc_dist = step_size * (j)
                wp_check  = [path[i][0] + (inc_dist*math.cos(zeta)),path[i][1] + (inc_dist*math.sin(zeta))] 
                if not self.is_valid(wp_check):
                    #print("Path is not valid")
                    return False 
        # For the waypoint itself check if `is_valid`.
            wp_check = path[i+1]
            
            if not self.is_valid(wp_check):
               # print("Path is not valid")
                return False 
        return True
            
    def is_inside_map(self, p):
        return bool(self.__position_to_map__(p))

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        
        # find p in image plane
        p_img  = p - self.origin
        
        # find row and col
        m = [int(np.floor(p_img[0] / self.resolution)),int(np.floor(p_img[1] / self.resolution))]
        # check that the cell is inside the grid map
        if m[0] >= self.map.shape[0] or m[1] >= self.map.shape[1] or m[0] < 0 or m[1] < 0:
            return None
        
        return m
    
    def __map_to_position__(self, m):
        if m[0] >= self.map.shape[0] or m[1] >= self.map.shape[1] or m[0] < 0 or m[1] < 0:
            return None
        position = [m[0] * self.resolution + self.origin[0],m[1] * self.resolution + self.origin[1]]  #+ resolution / 2
        return position