#!/usr/bin/env python2


import xml.etree.ElementTree as ET
import numpy as np
from obstacle import Obstacle


# ToDo: Use obstacleMsg instead of Obstacle
def read_gazebo_xml(filename, obstacle_prefix='obs_'):
    # obstacle_prefix = 'obs_'

    root_node = ET.parse(filename).getroot()
    models = root_node.findall('world/model')
    # model_state = root_node.findall('world/state/model')

    allObstacles = list()

    for model in models:
        if model.attrib['name'].startswith(obstacle_prefix) or model.attrib['name'].startswith('unit'):
            newObstacle = Obstacle(model.attrib['name'])

            model_state = root_node.find('world/state/model[@name="{0}"]'.format(model.attrib['name']))
            newObstacle.pose = np.fromstring(model_state.find('pose').text, dtype=float, sep=' ')

            geometryFirstChild = model.find('link/collision/geometry')[0]
            newObstacle.geometry = geometryFirstChild.tag
            if newObstacle.geometry == 'cylinder':
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text + ' ' + geometryFirstChild[1].text,
                                                dtype=float,
                                                sep=' ').tolist()
            else:
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text, dtype=float, sep=' ').tolist()

            newObstacle.typeOfObstacle = model.attrib['name'].split('_')[1]

            allObstacles.append(newObstacle.to_rosmsg())

    return allObstacles
