# Set map and model
param map = localPath('./Town01.xodr')
param carla_map = 'Town01'
param use2DMap = True

model scenic.simulators.carla.model

# Constants
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 10
EGO_BRAKING_THRESHOLD = 5
BRAKE_ACTION = 1.0

import sys


# Road and spawn point
lane = Uniform(*network.lanes)
egoPoint = new OrientedPoint on lane.centerline


# CarlaCola vehicle location 
emergencyPoint = new OrientedPoint following roadDirection from egoPoint for +10

# Creation of CarlaCola vehicle with follow lane behavior
emergency = new Car on emergencyPoint,
        with blueprint "vehicle.carlamotors.carlacola",
        with rolename "emergency",
        with behavior FollowLaneBehavior(8)


# Ego vehicle creation
ego = new Car on egoPoint,
        with blueprint EGO_MODEL,
        with rolename "hero"

# Termination condition
terminate after 15 seconds
