import numpy as np
from polygon import Polygon
import visualize_path
import path_plannig

def main()->int:
    edges = [(np.array([500,200]), np.array([900, 250])), (np.array([900,250]), np.array([940, 550])), (np.array([940,550]), np.array([750, 650])), 
    (np.array([750,650]), np.array([450, 550])), (np.array([450, 550]), np.array([500,200]))]
    edges2 = [(np.array([300,200]), np.array([1100, 300])), (np.array([1100, 300]), np.array([900, 1000])), 
    (np.array([900, 1000]), np.array([200, 1150])), (np.array([200, 1150]), np.array([300, 200]))]
    area = Polygon(edges)
    visualize_path.draw_polygon(area)
    path = path_plannig.path_planning(area, 5.0, (640, 480), np.pi/4, 0.0)
    v = np.array([800, 100])
    print(np.linalg.norm(v))
    return 0

if __name__ == "__main__":
    main()
    exit()