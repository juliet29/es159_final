import numpy as np
"Helper Functions"

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