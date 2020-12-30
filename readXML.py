import xml.etree.ElementTree as ET
import numpy as np

obstacle_prefix = 'obs_'


class Obstacle:
    def __init__(self, name):
        self.name: str = name
        self.pose: np.array = np.zeros(6)  # x y z alpha beta gamma
        self.geometry: str = 'box'  # box, cylinder, sphere
        self.dim: np.array = np.zeros(3)  # box: Länge x Breite x Höhe, cylinder: Radius x Height, sphere: Radius
        self.typeOfObstacle: str = 'default'  # tree, house, street, powerline, car, bus, bridge, statue, etc.


root_node = ET.parse('/home/daniel/OneDrive/APF - Erste Implementierung/gazebo_xml_test').getroot()
models = root_node.findall('world/model')

allObstacles = list()

for model in models:
    if model.attrib['name'].startswith(obstacle_prefix):
        newObstacle = Obstacle(model.attrib['name'])
        newObstacle.pose = np.fromstring(model.find('pose').text, dtype=float, sep=' ')
        geometryFirstChild = model.find('link/collision/geometry')[0]
        newObstacle.geometry = geometryFirstChild.tag
        if newObstacle.geometry == 'cylinder':
            newObstacle.dim = np.fromstring(geometryFirstChild[0].text + ' ' + geometryFirstChild[1].text, dtype=float,
                                            sep=' ')
        else:
            newObstacle.dim = np.fromstring(geometryFirstChild[0].text, dtype=float, sep=' ')

        newObstacle.typeOfObstacle = model.attrib['name'].split('_')[1]

        allObstacles.append(newObstacle)