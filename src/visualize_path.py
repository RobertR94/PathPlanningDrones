import matplotlib.pyplot as plt
import numpy as np

from polygon import Polygon


def draw_polygon(polygon : Polygon):
    for edge in polygon.edges:
        plt.plot([polygon.vertices[edge[0]][0], polygon.vertices[edge[1]][0]], [polygon.vertices[edge[0]][1], polygon.vertices[edge[1]][1]], color="blue")

    plt.show()
    return


def draw_path(polygon: Polygon, path : list, projected_area, scan_direction, up_direction):
    for edge in polygon.edges:
        plt.plot([polygon.vertices[edge[0]][0], polygon.vertices[edge[1]][0]], [polygon.vertices[edge[0]][1], polygon.vertices[edge[1]][1]], color="blue")

    for point in path:
        plt.scatter([point[0]], [point[1]], color="red")
        edges = [[point + (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction, 
        point + (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction],
        [point + (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction,
        point - (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction],
        [point - (projected_area[0]/2) * up_direction - (projected_area[1]/2)*scan_direction,
        point - (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction],
        [point - (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction,
        point + (projected_area[0]/2) * up_direction + (projected_area[1]/2)*scan_direction]]
        
        for edge in edges:
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color="green", linestyle='dashed')

    plt.show()
    return