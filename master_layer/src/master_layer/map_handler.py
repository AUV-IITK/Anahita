#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point, Pose

class MapHandler (object):
    # a class to provide the necessary information about the bot in the global fram
    # stores the global direction, the tasks approximate locations and much more

    def __init__(self):
        self.scale_x = 4.55
        self.scale_y = 5.54
        self.target_location = {
            'gate' : [],
            'path_marker1' : [],
            'path_marker2' : [],
            'tri_buoy' : [],
            'flat_buoy' : [],
            'dropper_bins' : [],
            'torpedo' : [],
            'octagon' : []
        }

        self.target_grid_location = {
            'gate' : [],
            'path_marker1' : [],
            'path_marker2' : [],
            'tri_buoy' : [],
            'flat_buoy' : [],
            'dropper_bins' : [],
            'torpedo' : [],
            'octagon' : []
        }

        self.occupancy_map = numpy.zeros(20, 11)

    def get_grid_coord (self, pose):
        point = pose.position
        x = int (point.x/self.scale_x)
        y = int (point.y/self.scale_y)

        return x, y

    def get_target_location (self, target):
        return self.target_location[target]

    def get_grid_location (self, grid_coord):
        x = (grid_coord[0] - 10)*self.scale_x - self.scale_x/2
        y = grid_coord[1]*self.scale_y - self.scale_y/2

        return x, y

    def get_path (self, intial_point, final_point):
        pass

    def is_occupied (self, grid_coord):
        return self.occupancy_map[grid_coord[0]][grid_coord[1]]
