import numpy as np
import random
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from plot_apf import PlotObstacle

# path to filename
ptf = "/home/leonard/catkin_ws/src/path_planning_private/benchmarks/scenarios/2d/2d_scenario_final.xml"

# parameters of the obstacle course
x_factor = y_factor = z_factor = 1
rho0 = 20
classification = 15
# goal coordinate
goal_coordinate = [18, 0, 0.5]

# parameters of attractiveness of trench
kb = 1.5

# parameters of attractiveness of goal
ka = 0.5

pts0 = 100
xlim = [-1, 20]
ylim = [-5, 5]
# xlim = [-5, 20]
# ylim = [-5, 5]
x, y = np.linspace(xlim[0], xlim[1], pts0), np.linspace(ylim[0], ylim[1], pts0)
X, Y = np.meshgrid(x, y)
pot0 = np.zeros((pts0, pts0))
vec = np.zeros((pts0, pts0, 2))
noo = 0


# Read xml file for obstacles
def read_gazebo_xml(filename, obstacle_prefix='obs_'):
    # obstacle_prefix = 'obs_'
    root_node = ET.parse(filename).getroot()
    models = root_node.findall('world/model')
    unit_prefix = 'unit'

    allObstacles = list()

    for model in models:
        if model.attrib['name'].startswith(obstacle_prefix) or model.attrib['name'].startswith(unit_prefix):
            newObstacle = PlotObstacle(model.attrib['name'])
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

            allObstacles.append(newObstacle)

    return allObstacles


def get_trench_field(arr, goal_coordinate):
    gradient = np.zeros(arr.shape)
    kb = 1
    position = np.array([0, 0, 0])
    goal_position = goal_coordinate
    uav_positions = arr
    help_vector = uav_positions-position
    line_vector = goal_position-position
    transp_help_vector = help_vector
    number_one = transp_help_vector[1]
    term_1 = line_vector[2] * transp_help_vector[:, 1] - line_vector[1] * transp_help_vector[:, 2]
    term_2 = line_vector[0] * transp_help_vector[:, 2] - line_vector[2] * transp_help_vector[:, 0]
    term_3 = line_vector[1] * transp_help_vector[:, 0] - line_vector[0] * transp_help_vector[:, 1]
    distance_line_vector = np.linalg.norm(line_vector)
    zaehler_grad_1 = -1 * line_vector[2] * term_2 * kb + line_vector[1] * term_3 * kb
    nenner_grad_1 = (term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5
    gradient[:, 0] = gradient[:, 0] + (
            -1 * line_vector[2] * term_2 * kb + line_vector[1] * term_3 * kb) / (np.linalg.norm(
        line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
    gradient[:, 1] = gradient[:, 1] + (
            line_vector[2] * term_1 * kb - line_vector[0] * term_3 * kb) / (np.linalg.norm(
        line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
    gradient[:, 2] = gradient[:, 2] + (
            -1 * line_vector[1] * term_1 * kb + line_vector[0] * term_2 * kb) / (np.linalg.norm(
        line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))

    return gradient

def get_goal_gradient(arr, goal_coordinate):
    gradient = np.zeros(arr.shape)
    transp_difference = arr
    difference = transp_difference - goal_coordinate
    distance = np.linalg.norm(difference, axis=1)
    gradient[:, 0] = gradient[:, 0] + ka * (difference[:, 0] / distance)
    gradient[:, 1] = gradient[:, 1] + ka * (difference[:, 1] / distance)
    gradient[:, 2] = gradient[:, 2] + ka * (difference[:, 2] / distance)
    return gradient

# All functions which are being used to determine the gradient
obs_list = read_gazebo_xml(ptf)

for i in range(len(obs_list)):
    for row in range(pts0):
        xrow = X[row]
        yrow = Y[row]
        arr = np.zeros((pts0, 3))
        arr[:, 0] = xrow
        arr[:, 1] = yrow
        vec[row] = vec[row] + obs_list[i].obstacle_plot_potential_function(arr, der=False)[:, :2]

for row in range(pts0):
    xrow = X[row]
    yrow = Y[row]
    arr = np.zeros((pts0, 3))
    arr[:, 0] = xrow
    arr[:, 1] = yrow
    # pot0[row] = get_potential_field(arr, goal_coordinate=goal_pos1)
    vec[row] = vec[row] + get_trench_field(arr, goal_coordinate=goal_coordinate)[:, :2]

for row in range(pts0):
    xrow = X[row]
    yrow = Y[row]
    arr = np.zeros((pts0, 3))
    arr[:, 0] = xrow
    arr[:, 1] = yrow
    # pot0[row] = get_potential_field(arr, goal_coordinate=goal_pos1)
    vec[row] = vec[row] + get_goal_gradient(arr, goal_coordinate=goal_coordinate)[:, :2]

# Plot gradient field
lim_vector_length = 3
vec = vec[:, :, :2]  # remove Z
for i in range(vec.shape[0]):
    for j in range(vec.shape[1]):
        vec_len = np.linalg.norm(vec[i,j])
        if vec_len > lim_vector_length:
            vec[i,j] *= lim_vector_length/vec_len

vec = vec * -1
fig0, ax0 = plt.subplots(1, figsize=(7, 3.75), dpi=320)
ax0.quiver(X, Y, vec[:, :, 0], vec[:, :, 1], cmap='gist_earth')
marker_one, = plt.plot(18, 0, marker='x', color='r', mew=5, ms=10)
marker_two, = plt.plot(0, 0, marker='x', color='b', mew=5, ms=10)
plt.ylabel('y')
plt.xlabel('x')
plt.legend(handles=[marker_one, marker_two], labels=['Goal', 'Start'], title='Legend', loc='upper center', bbox_to_anchor=(0.5,-0.1))
plt.title('Overall gradient field')
#plt.show()
fig0.savefig('Overall gradient field.svg', bbox_inches='tight')
#fig0.savefig('Trench gradient field.pdf')