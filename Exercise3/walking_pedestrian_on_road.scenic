## SET MAP AND MODEL
param map = localPath('./Town01.xodr')
param carla_map = 'Town01'
param use2DMap = True

model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz_2017"
PEDESTRIAN_MODEL = "walker.pedestrian.0001"
MARGIN_MOVMENT_x = Range(-2, -1)
MARGIN_MOVMENT_y = Range(10, 30)


import sys

lane = Uniform(*network.lanes)

## SPAWN POINTS
refPoint = new OrientedPoint on lane.centerline, facing roadDirection


## EGO VEHICLE CREATION
ego = new Car on refPoint,
        with blueprint EGO_MODEL,
        with rolename "hero"

## CREATION OF PEDESTRIAN (Pedestrian is created on a random point ahead of the ego vehicle, and walks forward)
pedestrian = new Pedestrian right of ego by (MARGIN_MOVMENT_x, MARGIN_MOVMENT_y),
       with blueprint PEDESTRIAN_MODEL,
       with regionContainedIn None,
       with behavior WalkForwardBehavior()

## AVOID INTERSECTIONS
require (distance to intersection) > 30 

## TERMINATION CONDITION
terminate after 15 seconds