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
PEDESTRIAN_MODEL = "walker.pedestrian.0001"


import sys

behavior PedestrianBehavior():
    do CrossingBehavior(ego, threshold=12)

## CURVED LANE FILTERING
def isCurved(lane):
    pts_raw = lane.centerline.points
    pts = [Vector(*point) for point in pts_raw]

    if len(pts) < 3:
        return False

    v1 = pts[1] - pts[0]
    v2 = pts[2] - pts[1]
    angle = abs(v1.angleTo(v2))

    return angle > 20 deg

## CURVED LANE SAMPLE
curvedLanes = [l for l in network.lanes if isCurved(l)]
curvedLane = Uniform(*curvedLanes)
lane = curvedLane

## SPAWN POINTS
refPoint = new OrientedPoint on lane.centerline, facing roadDirection
ego_point = new OrientedPoint following roadDirection from refPoint for Range(-22, -15)

## SPAWN POINTS FOR PEDESTRIAN 
crossingPoint = new OrientedPoint following roadDirection from ego_point for 15
crossingPoint = crossingPoint offset by (3, -2)  # Shift to side of road

## CREATION OF EGO VEHICLE
ego = new Car on ego_point,
        with blueprint EGO_MODEL,
        with rolename "hero"

## CREATION OF PEDESTRIAN
pedestrian = new Pedestrian on crossingPoint,
        facing 90 deg relative to ego_point.heading,
        with blueprint PEDESTRIAN_MODEL,
        with regionContainedIn None,
        with behavior PedestrianBehavior()
        

## AVOID INTERSECTIONS
require (distance to intersection) > 50  

## TERMINATION CONDITION
terminate after 15 seconds
