import numpy as np
import copy
import visualize_path

from polygon import Polygon

#computes the max height possible in order to achive a minmal given spatial resolution
#spatial resolution in mm(5-7mm are common)
def compute_max_height(resolution, spatial_resolution, angle_of_view)->float:
    max_height = resolution[0] / (2*spatial_resolution * np.tan(angle_of_view/2))
    return max_height

#computes the area captured from the camera at a specific height
def compute_projected_area(height, resolution, angle_of_view):
    projected_area = [0.0, 0.0]
    projected_area[0] = 2*height*np.tan(angle_of_view/2)
    projected_area[1] = projected_area[0] * (resolution[1]/resolution[0])
    return projected_area

#find longest edge
def get_longest_edge(edges, vertices):
    longest_edge = 0
    v1 = np.array([0.0, 0.0])
    for i, edge in enumerate(edges):
        v2 = vertices[edge[1]] - vertices[edge[0]]
        if np.linalg.norm(v2) > np.linalg.norm(v1):
            longest_edge = i
            v1 = v2

    return longest_edge

#find scan direction parallel to the longest bounding line
def get_scan_direction(longest_edge, vertices, edges)->np.array:
    return vertices[edges[longest_edge][1]] - vertices[edges[longest_edge][0]]

#find the vertex that has the longest distance to the longest bounding edge and compute the distance
def get_longest_distance(vertices, edge, edges):
    longest_distance = 0
    v1 = vertices[edges[edge][0]]
    v2 = vertices[edges[edge][1]]
    for vert in vertices:
        distance = np.abs((v2[0]-v1[0])*(v1[1]-vert[1])- (v1[0]-vert[0])*(v2[1]-v1[1]))/np.linalg.norm(v2-v1)
        if distance > longest_distance:
            longest_distance = distance
            
    return longest_distance

#get intercection of two lines represnted by two points
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

#get involved edge and boarder intersection
def update_boarder_points(area : Polygon, current_edge, index, point, scan_direction, up_direction, projected_area):
    cameera_edges = [[point + (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction, 
        point + (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction],
        [point + (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction,
        point - (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction],
        [point - (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction,
        point + (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction],
        [point - (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction,
        point - (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction]]

    for edge1 in cameera_edges:
        for edge in area.edges:
            edge2 = [area.vertices[edge[0]], area.vertices[edge[1]]]
            intersection_point = intersect(edge1, edge2)
            if intersection_point[0] != None:
                current_edge[index] = edge
                return intersection_point
    return False

#overlap: amount of overlap ranging from 0-1
def path_planning(area : Polygon, spatial_resolution : float, resolution, angle_of_view : float, overlap : float):
    way_points = list()

    height = compute_max_height(resolution, spatial_resolution, angle_of_view)
    projected_area = compute_projected_area(height, resolution, angle_of_view)
    longest_edge = get_longest_edge(area.edges, area.vertices)
    scan_direction = get_scan_direction(longest_edge, area.vertices, area.edges)
    scan_direction_normed = scan_direction/np.linalg.norm(scan_direction)
    up_direction_normed = np.array([-scan_direction_normed[1], scan_direction_normed[0]])
    longest_distance = get_longest_distance(area.vertices, longest_edge, area.edges)

    #d_x, d_y distance between centers of two adjacent areas in x and y direction
    d_x = projected_area[0]*(1-overlap)
    d_y = projected_area[1]*(1-overlap)

    number_of_stripes = int(np.ceil(longest_distance/d_x))
    if number_of_stripes%2 != 0:
        number_of_stripes += 1
    
    current_egeds = [longest_edge-1, longest_edge+1]
    current_vertices = [area.edges[current_egeds[0]][1], area.edges[current_egeds[1]][0]]
    current_angles = [area.angles[current_vertices[0]], area.angles[current_vertices[1]]]
    boarder_points = [copy.deepcopy(area.vertices[current_vertices[0]]), copy.deepcopy(area.vertices[current_vertices[1]])]
    up_shift = up_direction_normed * (projected_area[0]/2)
    x_overlap = 0.0
     #distance between stripes
    delta_x = (longest_distance - projected_area[0])/(number_of_stripes-1)

    for stripe in range(number_of_stripes):
        start_index = 0
        stop_index = 1

        point_shift = 0.0
        stripe_length = np.linalg.norm(boarder_points[1]-boarder_points[0])


        if current_angles[start_index] > np.pi/2:
            stripe_length += (1/np.tan(np.pi-current_angles[start_index])) * (projected_area[0]-x_overlap)
            point_shift = (1/np.tan(np.pi-current_angles[start_index])) * (projected_area[0]-x_overlap) * (-1*scan_direction_normed)
        if current_angles[stop_index] > np.pi/2:
            stripe_length += 1/np.tan(np.pi-current_angles[stop_index]) * projected_area[0]

        number_of_waypoints = int(np.ceil(stripe_length/d_y))
        #distance between waypoints
        delta_y = (stripe_length-projected_area[1])/(number_of_waypoints-1)

        x_overlap = d_x-delta_x

        way_point = boarder_points[start_index] + point_shift + (scan_direction_normed*(projected_area[1]/2)) + up_shift
        way_points.append(copy.deepcopy(way_point))
        boarder_points[start_index] = update_boarder_points(area, current_egeds, start_index, way_point, scan_direction_normed, up_direction_normed, projected_area) + (-1*up_direction_normed * projected_area[0]/2)
        for i in range(number_of_waypoints-1):
            way_point += scan_direction_normed * delta_y
            way_points.append(copy.deepcopy(way_point))
        
        boarder_points[stop_index] = update_boarder_points(area, current_egeds, stop_index, way_point, scan_direction_normed, up_direction_normed, projected_area) + (-1*up_direction_normed * projected_area[0]/2)
        scan_direction = scan_direction * (-1)
        start_index = stop_index
        stop_index = start_index
        up_shift = up_direction_normed *delta_x
 #       up_shift = up_direction_normed * d_x

    visualize_path.draw_path(area, way_points, projected_area, scan_direction_normed, up_direction_normed)
    return way_points

    

    

    



