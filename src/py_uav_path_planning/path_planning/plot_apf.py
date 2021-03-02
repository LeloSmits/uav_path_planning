import numpy as np
import matplotlib.pyplot as plt
import random
import xml.etree.ElementTree as ET
import pickle

# ptf = "/home/leonard/catkin_ws/src/path_planning_private/benchmarks/scenarios/2d/2d_scenario_our_drone.xml"
# ptf = "/home/leonard/catkin_ws/src/path_planning_private/benchmarks/scenarios/2d/2d_scenario.xml"
ptf = "/home/daniel/catkin_ws/src/path_planning_private/benchmarks/scenarios/2d/2d_scenario_final_3.xml"

pickle_file = "/home/daniel/path_planning_logs/Leo/path_log_scenario_3_ours.p"
pickle_file_px4 = "/home/daniel/path_planning_logs/Leo/path_log_px4_scenario_3.p"
# open pickle file
file_open = open(pickle_file, "rb")
pickle_data = pickle.load(file_open)
pos_data = pickle_data['position_data']
meta_data = pickle_data['meta_data']

time_total = pos_data[-1, 0] - pos_data[0, 0]

distance_from_path = np.zeros(pos_data.shape[0])

# index 1 to index 3 for uav position important
uav_pos = np.array(pos_data[:, 1:4])
wp_next = np.array(pos_data[:, 4:7])
wp_prev = np.array(pos_data[:, 7:10])

# open PX4 file
file_open = open(pickle_file_px4, "rb")
pickle_data = pickle.load(file_open)
pos_data = pickle_data['position_data']

# index 1 to index 3 for uav position important
uav_pos_px4 = np.array(pos_data[:, 1:4])


x_factor = y_factor = z_factor = 1
rho0 = 20
classification = 30


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


class PlotObstacle:
    def __init__(self, name):
        self.name = name
        self.pose = list()
        self.geometry = 'box'
        self.dim = list()
        self.typeOfObstacle = 'default'  # Tree, house, street, powerline, car, bus
        self.classification = int()
        self.a = float()
        self.b = float()
        self.c = float()
        self.potential = float()

    def obstacle_plot_potential_function(self, arr, der):

        if self.geometry == 'box':
            self.a = self.dim[0] * x_factor
            self.b = self.dim[1] * y_factor
            self.c = self.dim[2] * z_factor
        elif self.geometry == 'cylinder':
            self.a = self.dim[0] * x_factor
            self.b = self.a
            self.c = self.dim[1] * z_factor
        elif self.geometry == 'sphere':
            self.a = self.dim[0] * x_factor
            self.b = self.a
            self.c = self.a * z_factor  # * 2


        waypoint = arr
        close = 10 ** -8
        x_dist = waypoint[:, 0] - self.pose[0]
        y_dist = waypoint[:, 1] - self.pose[1]
        #z_dist = waypoint[2] - self.pose[2]
        z_dist = 0
        mask_1 = abs(x_dist) < close
        mask_2 = abs(y_dist) < close
        mask_3 = abs(z_dist) < close
        x_dist[mask_1] = 10 ** -8
        y_dist[mask_2] = 10 ** -8
        # z_dist[mask_3] = 10 ** -8
        self.classification = 10
        if der:
            potential_test = .5 * self.classification * (1 /
                                                         ((x_dist ** 2) / self.a ** 2
                                                          + (y_dist ** 2) / self.b ** 2
                                                          + (z_dist ** 2) / self.c ** 2) - 1 / rho0 ** 2)
            return potential_test
        elif not der:
            gradient = np.zeros(arr.shape)
            gradient_mask = x_dist**2 + y_dist**2 + z_dist**2 < rho0**2
            gradient[:, 0] = ((-1 * self.classification * x_dist) / (self.a ** 2)) / \
                     ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
            gradient[:, 1] = ((-1 * self.classification * y_dist) / (self.b ** 2)) / \
                     ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
            gradient[:, 2] = ((-1 * self.classification * z_dist) / (self.c ** 2)) / \
                     ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
            # gradient[gradient_mask, 0] = 0
            # gradient[gradient_mask, 1] = 0
            # gradient[gradient_mask, 2] = 0
            return gradient


# Step 1: Take one obstacle and plot the potential
# Step 2: Take all obstacles and iterate over them to plot their potential

goal_coordinate = [18, 0, 0.5]

# parameters of attractiveness of trench
kb = 0.01

# parameters of attractiveness of goal
ka = 1

obs_list = read_gazebo_xml(ptf)


# This is not done in combination with gazebo. Values are chosen randomly

def obstacle_potential_function(obs123_pose, obs123_a, obs123_b, obs123_c, arr):
    waypoint = arr
    close = 10 ** -8
    x_dist = waypoint[:, 0] - obs123_pose[0]
    y_dist = waypoint[:, 1] - obs123_pose[1]
    z_dist = 0
    mask_1 = abs(x_dist) < close
    mask_2 = abs(y_dist) < close
    mask_3 = abs(z_dist) < close
    x_dist[mask_1] = 10 ** -8
    y_dist[mask_2] = 10 ** -8
    # z_dist[mask_3] = 10 ** -8
    potential_test = .5 * classification * (1 /
                                             ((x_dist ** 2) / obs123_a ** 2
                                              + (y_dist ** 2) / obs123_b ** 2
                                              + (z_dist ** 2) / obs123_c ** 2) - 1 / rho0 ** 2)
    return potential_test

def trench_potential(arr, goal_coordinate):
    position = np.array([[0, 0, 0]])
    goal_position = goal_coordinate
    uav_positions = arr
    help_vector = uav_positions-position
    line_vector = goal_position-position
    dist2 = np.linalg.norm(np.cross(help_vector, line_vector), axis=1)/np.linalg.norm(line_vector)
    potential = kb * dist2
    return potential

def attractive_goal_potential(arr, goal_coordinate):
    goal_point = goal_coordinate
    transp_pot_coord = arr
    difference = transp_pot_coord - goal_point
    dist = np.linalg.norm(difference, axis=1)
    potential2 = ka * dist
    return potential2


# goal_pos1 = [25, 5, 0]
pts0 = 500
xlim = [-1, 25]
ylim = [-10, 10]
x, y = np.linspace(xlim[0], xlim[1], pts0), np.linspace(ylim[0], ylim[1], pts0)
X, Y = np.meshgrid(x, y)
pot0 = np.zeros((pts0, pts0))
vec = np.zeros((pts0, pts0, 2))
noo = 0


while noo < 5:
    obs123_pose = [random.randint(-4, 8), random.randint(-4, 8), random.randint(0, 1)]
    print(obs123_pose)
    obs123_a = obs123_b = obs123_c = 1
    for row in range(pts0):
        xrow = X[row]
        yrow = Y[row]
        arr = np.zeros((pts0, 3))
        arr[:, 0] = xrow
        arr[:, 1] = yrow
        # pot0[row] = get_potential_field(arr, goal_coordinate=goal_pos1)
        # pot0[row] = pot0[row] + obstacle_potential_function(obs123_pose, obs123_a, obs123_b, obs123_c, arr[:, :2])
    noo = noo + 1


# Iterate over obstacles which were found in the gazebo xml
for i in range(len(obs_list)):
    for row in range(pts0):
        xrow = X[row]
        yrow = Y[row]
        arr = np.zeros((pts0, 3))
        arr[:, 0] = xrow
        arr[:, 1] = yrow
        pot0[row] = pot0[row] + obs_list[i].obstacle_plot_potential_function(arr, der=True)


for row in range(pts0):
    xrow = X[row]
    yrow = Y[row]
    arr = np.zeros((pts0, 3))
    arr[:, 0] = xrow
    arr[:, 1] = yrow
    pot0[row] = pot0[row] + trench_potential(arr, goal_coordinate)

for row in range(pts0):
    xrow = X[row]
    yrow = Y[row]
    arr = np.zeros((pts0, 3))
    arr[:, 0] = xrow
    arr[:, 1] = yrow
    pot0[row] = pot0[row] + attractive_goal_potential(arr, goal_coordinate)



lim_pot = 50
pot0[np.where(pot0 > lim_pot)] = lim_pot


# ------- PLOT
fig0, ax0 = plt.subplots(1, figsize=(7, 3.75), dpi=320)
# fig0.add_axes
cs = ax0.contour(X, Y, pot0, cmap='gist_earth', levels=20)
plot_apf, = ax0.plot(uav_pos[:, 0], uav_pos[:, 1], label="APF", color="green")
plot_vfh, = ax0.plot(uav_pos_px4[:,0], uav_pos_px4[:,1], label="VFH", color="orange")
ax0.clabel(cs, inline=1, fontsize=10)
marker_one = ax0.scatter(18, 0, marker='x', color='r', linewidth=5, s=100, zorder=10)
marker_two = ax0.scatter(0, 0, marker='x', color='b', linewidth=5, s=100, zorder=10)
plt.ylabel('y')
plt.xlabel('x')
plt.legend(handles=[marker_one, marker_two, plot_apf, plot_vfh], labels=['Goal', 'Start', 'APF-Path', '3DVFH*-Path'],
           title='Legend', loc='best', bbox_to_anchor=(1.3, 1))
plt.title('2D - Scenario 3 - Flown Path')
#plt.show()
plt.savefig('2D - Scenario 3 - Flown Path.svg', bbox_inches='tight')
# fig0.savefig('Potentialfield_with_trench_goal_obstacles.pdf')
# ax0.scatter(goal_pos0[0], goal_pos0[1], label="Start")
# ax0.scatter(goal_pos1[0], goal_pos1[1], label="Goal")
# ax0.scatter(uav_pos0[0], uav_
