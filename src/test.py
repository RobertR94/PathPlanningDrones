import numpy as np

e1 = [np.array([2, 2]), np.array([4, 4])]
e2 = [np.array([4, 2]), np.array([2, 4])]

def line_intersection(e1, e2):
    a = e1[0]
    b = e1[1]
    c = e2[0]
    d = e2[1]
    a1 = b[1] - a[1]
    b1 = a[0] - b[0]
    c1 = a1*(a[0]) + b1*(a[1])
 
    # Line CD represented as a2x + b2y = c2
    a2 = d[1] - c[1]
    b2 = c[0] - d[0]
    c2 = a2*(c[0]) + b2*(c[1])
 
    determinant = a1*b2 - a2*b1
 
    if (determinant == 0):
        # The lines are parallel. This is simplified
        # by returning a pair of FLT_MAX
        return np.array([None, None])
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        return np.array([x, y])

def change_array(a):
    a[1] = 0
    return

print(line_intersection(e1, e2))

a = [1, 2, 4]
change_array(a)
print(a)

def intersect(e1, e2):

    x1 = e1[0][0]
    y1 = e1[0][1]
    x2 = e1[1][0]
    y2 = e1[1][1]
    x3 = e2[0][0]
    y3 = e2[0][1]
    x4 = e2[1][0]
    y4 = e2[1][1]

    denominator = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

    if denominator == 0:
        return np.array([None, None])
    

    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator

    # is the intersection along the segments
    if ua < 0 or ua > 1 or ub < 0 or ub > 1:
        return np.array([None, None])
    

    # Return a object with the x and y coordinates of the intersection
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)

    return np.array([x, y])


print(intersect(e1, e2))