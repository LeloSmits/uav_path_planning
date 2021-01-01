#!/usr/bin/env python2


import xml.etree.ElementTree as ET
import numpy as np
from obstacle import Obstacle


def read_gazebo_xml(filename, obstacle_prefix='obs_'):
    # obstacle_prefix = 'obs_'

    root_node = ET.parse(filename).getroot()
    models = root_node.findall('world/model')

    allObstacles = list()

    for model in models:
        if model.attrib['name'].startswith(obstacle_prefix):
            newObstacle = Obstacle(model.attrib['name'])
            newObstacle.pose = np.fromstring(model.find('pose').text, dtype=float, sep=' ')
            geometryFirstChild = model.find('link/collision/geometry')[0]
            newObstacle.geometry = geometryFirstChild.tag
            if newObstacle.geometry == 'cylinder':
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text + ' ' + geometryFirstChild[1].text,
                                                dtype=float,
                                                sep=' ').tolist()
            else:
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text, dtype=float, sep=' ').tolist()

            newObstacle.typeOfObstacle = model.attrib['name'].split('_')[1]

            allObstacles.append(newObstacle)

    return allObstacles
