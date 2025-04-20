## SET MAP AND MODEL
param map = localPath('./Town01.xodr')
param carla_map = 'Town01'
param use2DMap = True

model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 10
EGO_BRAKING_THRESHOLD = 8


BRAKE_ACTION = 1.0

import sys


## ROAD & SPAWN POINT
lane = Uniform(*network.lanes)

## CREATION OF EGO VEHICLE
egoPoint = new OrientedPoint on lane.centerline

ego = new Car on egoPoint,
        with blueprint EGO_MODEL,
        with rolename "hero"

## REQUIREMENT TO BE IN THE ENTRANCE OF AN INTERSECTION
require 15 < (distance to intersection) < 25

## TERMINATION CONDITION
terminate when (distance to intersection) < 0
terminate after 20 seconds
