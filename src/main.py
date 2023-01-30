import numpy as np
from polygon import Polygon
import visualize_path
import path_plannig
from http.server import BaseHTTPRequestHandler, HTTPServer
import time


def main(edges_server) -> int:
    edges = [(np.array([500, 200]), np.array([900, 250])), (np.array([900, 250]), np.array([940, 550])), (np.array([940, 550]), np.array([750, 650])),
    (np.array([750, 650]), np.array([450, 550])), (np.array([450, 550]), np.array([500, 200]))]
    edges2 = [(np.array([300, 200]), np.array([1100, 300])), (np.array([1100, 300]), np.array([900, 1000])),
    (np.array([900, 1000]), np.array([200, 1150])), (np.array([200, 1150]), np.array([300, 200]))]
    area = Polygon(edges_server)
    visualize_path.draw_polygon(area)
    path = path_plannig.path_planning(area, 5.0, (640, 480), np.pi/4, 0.0)
    v = np.array([800, 100])
    print(np.linalg.norm(v))
    return 0


hostName = "localhost"
serverPort = 9090


class MyServer(BaseHTTPRequestHandler):
    def do_GET(self):
        params = self.path.split("=")[1]
        temp = params.split(",")
        coords = []
        for ele in temp:
            el1, el2 = ele.split(";")
            print(el1, el2)
            coords.append((np.array([float(el1) * 10, float(el2) * 10])))
        edges = []
        for i in range(len(coords)):
            edges.append((coords[i], coords[(i+1) % len(coords)]))

        main(edges)
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(
            bytes("<html><head><title>https://pythonbasics.org</title></head>", "utf-8"))
        self.wfile.write(bytes("<p>Request: %s</p>" % self.path, "utf-8"))
        self.wfile.write(bytes("<body>", "utf-8"))
        self.wfile.write(
            bytes("<p>This is an example web server.</p>", "utf-8"))
        self.wfile.write(bytes("</body></html>", "utf-8"))

if __name__ == "__main__":
    webServer=HTTPServer((hostName, serverPort), MyServer)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")
    # main()
    exit()
