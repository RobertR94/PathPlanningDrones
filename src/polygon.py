import numpy as np

class Polygon:

    def __init__(self, edges):
        self.edges = list()
        self.vertices = list()
        self.angles = list()
        if len(edges) > 0:
            self.create(edges)
            return 


        return 

    def compute_angle(self, v1, v2)->None:
        angle = np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
        self.angles.append(angle)
        return

    def create(self, edges: list)->None:
        self.vertices = [0 for e in edges]
        for i, edge in enumerate(edges):
            self.vertices[i] = edge[0]
            if i == len(edges)-1:
                self.edges.append((i, 0))
            else:
                self.edges.append((i, i+1))
            prev_edge = edges[i-1]
            self.compute_angle(prev_edge[0] - prev_edge[1], edge[1] - edge[0])

        return

    def get_vert_index(self, vert):
        for i, v in enumerate(self.vertices):
            if vert[0] == v[0] and vert[1] == v[1]:
                return i
        return False