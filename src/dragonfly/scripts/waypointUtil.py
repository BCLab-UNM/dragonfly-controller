#! /usr/bin/env python
import math, pulp
from enum import Enum
from dragonfly_messages.msg import LatLon
from geometry_msgs.msg import PoseStamped, Point

class Span(Enum):
    WALK = 1
    RANGE = 2

def createWaypoint(x, y, altitude):
    waypoint = PoseStamped()
    waypoint.pose.position.x = x
    waypoint.pose.position.y = y
    waypoint.pose.position.z = altitude

    return waypoint

def calculateRange(type, start, end, length):
    print "TYPE: {} {} {}".format(type, Span.WALK, Span.RANGE)
    if type == Span.WALK:
        waypoints = []
        print "Calculating walk"
        deltax = end.x - start.x
        deltay = end.y - start.y
        deltaz = end.z - start.z
        distance = math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))
        for i in range(1, int(distance / length) + 1):
            waypoints.append(Point(start.x + (i * length * deltax / distance),
                                   start.y + (i * length * deltay / distance),
                                   start.z + (i * length * deltaz / distance)))
        return waypoints
    elif type == Span.RANGE:
        return [end]

def buildRelativeWaypoint(localposition, position, waypoint, altitude):
    earthCircumference = 40008000
    return createWaypoint(
        localposition.x - ((position.longitude - waypoint.longitude) * (earthCircumference / 360) * math.cos(position.latitude * 0.01745)),
        localposition.y - ((position.latitude - waypoint.latitude) * (earthCircumference / 360)) ,
        altitude
    )

def createLatLon(localwaypoint, localposition, position):
    earthCircumference = 40008000
    latitude = position.latitude - (localposition.y - localwaypoint.y) * 360 / earthCircumference
    longitude= position.longitude - (localposition.x - localwaypoint.x) * 360 / (earthCircumference * math.cos(latitude * 0.01745))

    return LatLon(latitude = latitude, longitude = longitude, relativeAltitude = localwaypoint.z)

def build3DDDSAWaypoints(rangeType, stacks, size, index, loops, radius, steplength):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        ddsaWaypoints = buildDDSAWaypoints(rangeType, stack, size, index, loops, radius, steplength)
        if toggleReverse:
            ddsaWaypoints = ddsaWaypoints[::-1]
        waypoints = waypoints + ddsaWaypoints

        toggleReverse = not toggleReverse

    return waypoints

def buildDDSAWaypoints(rangeType, altitude, size, index, loops, radius, steplength):

    waypoints = []
    start = Point(0, 0, altitude)
    waypoints.append(start)
    previous = start
    for loop in range(0, loops):
        for corner in range(0, 4):

            if (loop == 0 and corner == 0):
                next = Point(0, index + 1, altitude)
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if (corner == 0):
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif (corner == 3):
                    xoffset = -xoffset
                if (corner == 2 or corner == 3):
                    yoffset = -yoffset

                next = Point(xoffset, yoffset, altitude)

            print "{}, {} -> {}, {}".format(previous.x, previous.y, next.x, next.y)

            for waypoint in calculateRange(rangeType, previous, next, steplength):
                waypoints.append(Point(waypoint.x * radius, waypoint.y * radius, waypoint.z))

            previous = next

    return waypoints

def linearXRange(points, setY, type):

    problem = pulp.LpProblem('range', type)

    x = pulp.LpVariable('x', cat='Continuous')
    y = pulp.LpVariable('y', cat='Continuous')

    # Objective function
    problem += x

    def buildLineEquation(index1, index2):
        a = -(points[index2][1] - points[index1][1])
        b = points[index2][0] - points[index1][0]
        c = (a * points[index1][0]) + (b * points[index1][1])
        # print '(', a, 'x+',b,'y >=',c,'),'
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem +=buildLineEquation(i-1, i)

    problem += buildLineEquation(len(points)-1, 0)

    problem += y == setY

    # print problem
    pulp.GLPK_CMD(msg=0).solve(problem)

    return x.value()

def linearYRange(points, type):

    problem = pulp.LpProblem('range', type)

    x = pulp.LpVariable('x', cat='Continuous')
    y = pulp.LpVariable('y', cat='Continuous')

    # Objective function
    problem += y

    def buildLineEquation(index1, index2):
        a = -(points[index2][1] - points[index1][1])
        b = points[index2][0] - points[index1][0]
        c = (a * points[index1][0]) + (b * points[index1][1])
        # print '(', a, 'x+',b,'y >=',c,'),'
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem +=buildLineEquation(i-1, i)

    problem += buildLineEquation(len(points)-1, 0)

    # print problem
    pulp.GLPK_CMD(msg=0).solve(problem)

    return y.value()

def build3DLawnmowerWaypoints(rangeType, altitude, localPosition, position, stacks, boundary, steplength):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        lawnmowerWaypoints = buildLawnmowerWaypoints(rangeType, altitude + stack, localPosition, position, boundary, steplength)
        if toggleReverse:
            lawnmowerWaypoints = lawnmowerWaypoints[::-1]
        waypoints = waypoints + lawnmowerWaypoints

        toggleReverse = not toggleReverse

    return waypoints

def buildLawnmowerWaypoints(rangeType, altitude, localposition, position, boundary, steplength):
    boundary_meters = []

    waypoints = []

    for waypoint in boundary:
        goalPos = buildRelativeWaypoint(localposition, position, waypoint, altitude)

        boundary_meters.append((goalPos.pose.position.x, goalPos.pose.position.y))


    # Get minimum in Y dimension
    miny = linearYRange(boundary_meters, pulp.LpMinimize)
    # Get maximum in Y dimension
    maxy = linearYRange(boundary_meters, pulp.LpMaximize)


    print "miny:{} maxy:{} ".format(miny, maxy)

    stepdirection = 1 if miny < maxy else -1

    for y in range(int(miny), int(maxy), int(2 * steplength)):
        minx = linearXRange(boundary_meters, y, pulp.LpMinimize)
        maxx = linearXRange(boundary_meters, y, pulp.LpMaximize)
        print "minx:{} maxx:{} ".format(minx, maxx)
        waypoints.append(createWaypoint(minx, y, altitude))
        for point in calculateRange(rangeType, Point(minx, y, altitude), Point(maxx, y, altitude), steplength):
            waypoints.append(createWaypoint(point.x, point.y, point.z))
        minx = linearXRange(boundary_meters, y + steplength, pulp.LpMinimize)
        maxx = linearXRange(boundary_meters, y + steplength, pulp.LpMaximize)
        print "minx:{} maxx:{} ".format(minx, maxx)
        waypoints.append(createWaypoint(maxx, y + steplength, altitude))
        for point in calculateRange(rangeType, Point(maxx, y + (stepdirection * steplength), altitude), Point(minx, y + (stepdirection * steplength), altitude), steplength):
            waypoints.append(createWaypoint(point.x, point.y, point.z))

    return waypoints