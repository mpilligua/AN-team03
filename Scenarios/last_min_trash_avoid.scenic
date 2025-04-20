## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('./Town01.xodr')
param carla_map = 'Town01'
param use2DMap = True

model scenic.simulators.carla.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_CAR_SPEED = 19
LEAD_CAR_TURNING_THRESHOLD = 15
DISTANCE_LEAD_TO_OBS = Range(-50, -45)
DISTANCE_EGO_TO_LEAD = Range(-15, -10)

import sys

## DEFINING LEAD CAR BEHAVIOR
## FollowLaneBehavior until reaches the obstacle, then changes lane to the left
behavior LeadCarBehavior(speed):
    changedLane = False
    try: 
        do FollowLaneBehavior(speed)

    interrupt when (distance from self to obstacle) < LEAD_CAR_TURNING_THRESHOLD:
        laneSectionToSwitch = network.laneSectionAt(self)

        if not changedLane:
            do LaneChangeBehavior(laneSectionToSwitch._laneToLeft, is_oppositeTraffic=True, target_speed=speed)
            do FollowLaneBehavior(speed, is_oppositeTraffic=True)
            changedLane = True
        else:
            wait
    
## LANES WITH LEFT LANE
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft != None:
            laneSecsWithLeftLane.append(laneSec)
SecLane = Uniform(*laneSecsWithLeftLane)

## SPAWN POINT
refPoint = new OrientedPoint on SecLane.lane.centerline,
            facing roadDirection

## SPAWN LOCATIONS FOR LEAD CAR
leadPoint = new OrientedPoint following roadDirection from refPoint for DISTANCE_LEAD_TO_OBS

leadCar = new Car on leadPoint,
        with behavior LeadCarBehavior(LEAD_CAR_SPEED)

## CREATION OF TRASH
obstacle = new Trash on refPoint

## SPAWN POINT FOR EGO VEHICLE
egoPoint = new OrientedPoint following roadDirection from leadCar for DISTANCE_EGO_TO_LEAD

## CREATION OF EGO VEHICLE
ego = new Car on egoPoint,
        with blueprint EGO_MODEL,
        with rolename "hero"


## AVOID INTERSECTIONS
require (distance to intersection) > 50

## TERMINATION CONDITION
terminate when (distance to refPoint) < 1
terminate after 15 seconds