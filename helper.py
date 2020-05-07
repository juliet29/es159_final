"Helper Functions"
from scipy.spatial import distance
import numpy as np


def midpoint(p1, p2):
    "Return the midpoint of 2 points in 3D space at z = 0"
    return (np.mean([p1[0], p2[0]]), np.mean([p1[1], p2[1]]), 0)

def boundBox(pos, v):
    "v should be an array of 8 floats"
    x = pos[0]
    y = pos[1]
    # define the vertices of the bounding box
    bb = [(x+v[0], y+v[1], 0),
    (x+v[2], y+v[3], 0),
    (x+v[4], y+v[5], 0),
    (x+v[6], y+v[7], 0)]
    # get the midpoint of the bounding box
    bbC = midpoint(bb[0], bb[2])
    return (bb, bbC)

def drawCont(p, arr, color):
    # add arbitrary item to the end 
    for i in range(len(arr) -1):
        p.addUserDebugLine(arr[i], arr[i+1], lineColorRGB=color, lineWidth=1, lifeTime=0)

def drawContDF(p, df, color):
    # add arbitrary item to the end 
    for point in range(df.shape[0] -1):
        currPoint = tuple(df.iloc[point])
        nextPoint = tuple(df.iloc[point + 1])
        p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=color, lineWidth=1, lifeTime=0)

def boundEdge(bb, var):
    "Determines the max and min boundaries of a bounding box in a var direction"
    ns = [bb[i][var] for i in range(len(bb))]
    n_max = max(ns)
    n_min = min(ns)
    return(n_max, n_min)

def pointFilter(df, bb):
    "Filters an x,y,z dataframe of point based on the coordinates of a bounding box "
    # points that are less than the max x
    a = df[df['x'] < boundEdge(bb, 0)[0]]
    # points that are greater than the min x
    b = a[a['x'] > boundEdge(bb, 0)[1]]
    # points that are less than the max y 
    c = b[b['y'] < boundEdge(bb, 1)[0] ]
    # points that are greater than the min y 
    d = c[c['y'] > boundEdge(bb, 1)[1] ]
    return d

def findFurthestPoint(df, initPt):
    "Takes in a dataframe of x,y,z points and a initial point defined as a list [x,y,z], and finds the furthest point in the dataframe from the initial point"
    # initial furthest point is that which is defined by the first row of the df
    furthestPt = df.iloc[0]
    for i in range(df.shape[0]):
        furthest_dist = distance.euclidean(initPt, furthestPt)
        # only replace the furthestPt if there is a greater furthest_dist
        if distance.euclidean(initPt, list(df.iloc[i])) > furthest_dist:
            furthestPt = df.iloc[i]
    return furthestPt

def findClosestPoint(df, initPt):
    "Takes in a dataframe of x,y,z points and a initial point defined as a list [x,y,z], and finds the furthest point in the dataframe from the initial point"
    # initial furthest point is that which is defined by the first row of the df
    closestPt = df.iloc[0]
    for i in range(df.shape[0]):
        closestDist = distance.euclidean(initPt, closestPt)
        # only replace the furthestPt if there is a greater furthest_dist
        if distance.euclidean(initPt, list(df.iloc[i])) < closestDist:
            closestPt = df.iloc[i]
    return closestPt
