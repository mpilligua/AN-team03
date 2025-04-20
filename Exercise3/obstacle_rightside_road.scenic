## SET MAP AND MODEL
param map = localPath('./Town01.xodr')
param carla_map = 'Town01'
param use2DMap = True

model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz_2017"
OBSTACLE_MODEL = "static.prop.streetbarrier" # "static.prop.shoppingcart"
MARGIN_MOVMENT_x = Range(-2, 2)
MARGIN_MOVMENT_y = Range(10, 30)
ANGLE_OBSTACLE = Range(180, 270)


import sys

## LANE SAMPLING
lane = Uniform(*network.lanes)

## SPAWN POINTS
refPoint = new OrientedPoint on lane.centerline, facing roadDirection


## CREATION OF EGO VEHICLE
ego = new Car on refPoint,
        with blueprint EGO_MODEL,
        with rolename "hero"

## CREATION OF BARRIER OBSTACLE
pedestrian = new Trash right of ego by (MARGIN_MOVMENT_x, MARGIN_MOVMENT_y),
       facing ANGLE_OBSTACLE,
       with blueprint OBSTACLE_MODEL


## AVOID INTERSECTIONS
require (distance to intersection) > 25  

## TERMINATION CONDITION
terminate after 25 seconds