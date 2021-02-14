import numpy as np

x_factor = y_factor = z_factor = 1

def _gradient_callback(self, message):
    print(message)
    waypoint = [5, 5, 0.5]
    mode = 0
    # print(waypoint)
    # Defining key variables and parameters
    x_coordinate = []
    y_coordinate = []
    z_coordinate = []
    pot_coordinates = []

    for i in (range(len(waypoint) / 3)):
        x_coordinate.append(waypoint[0 + i * 3])
        y_coordinate.append(waypoint[1 + i * 3])
        z_coordinate.append(waypoint[2 + i * 3])

    pot_coordinates.append(x_coordinate)
    pot_coordinates.append(y_coordinate)
    pot_coordinates.append(z_coordinate)
    # Conversion of the list of lists to a numpy array
    pot_coordinates = np.array(pot_coordinates, dtype=float)
    # print(pot_coordinates)

    obstacle_list = list()
    # print(self.current_obstacles)
    for i in range(len(self.current_obstacles)):
        obs_from_obstacle_map = self.current_obstacles[i]

        # Initiating a class instance for each current obstacle
        obs123 = Obstacle()
        obs123.name = obs_from_obstacle_map.name
        obs123.pose = obs_from_obstacle_map.pose
        obs123.geometry = obs_from_obstacle_map.geometry
        obs123.dim = obs_from_obstacle_map.dim
        obs123.typeOfObstacle = obs_from_obstacle_map.typeOfObstacle
        obs123.get_classification(df=self.df)

        if obs123.geometry == 'box':
            obs123.a = obs_from_obstacle_map.dim[0] * x_factor
            obs123.b = obs_from_obstacle_map.dim[1] * y_factor
            obs123.c = obs_from_obstacle_map.dim[2] * z_factor
        elif obs123.geometry == 'cylinder':
            obs123.a = obs_from_obstacle_map.dim[0] * x_factor
            obs123.b = obs123.a
            obs123.c = obs_from_obstacle_map.dim[1] * z_factor
        elif obs123.geometry == 'sphere':
            obs123.a = obs_from_obstacle_map.dim[0] * x_factor
            obs123.b = obs123.a
            obs123.c = obs123.a * z_factor  # * 2

        obstacle_list.append(obs123)
    my_length = len(waypoint)
    # Calculating the potential for each obstacle at each coordinate
    # potential = np.zeros()
    gradient = np.zeros(shape=(my_length / 3, my_length))
    for i in range(len(obstacle_list)):
        gradient = gradient + obstacle_list[i].obstacle_potential_function(uav_pose=self.next_waypoint,
                                                                           waypoint=pot_coordinates, der=True)
    if mode == 0:
        # Calculate the gradient due to the influence of the goal

        # ToDo
        goal_point = self.next_waypoint
        transp_pot_coord = pot_coordinates.transpose()
        print("This is trans_pot_coord" + str(transp_pot_coord))
        difference = transp_pot_coord - goal_point
        transp_difference = difference.transpose()
        distance = np.linalg.norm(difference, axis=1)
        # print(transp_difference)
        # print(distance)
        gradient[:, 0] = gradient[:, 0] + ka * (transp_difference[0] / distance)
        gradient[:, 1] = gradient[:, 1] + ka * (transp_difference[1] / distance)
        gradient[:, 2] = gradient[:, 2] + ka * (transp_difference[2] / distance)
        # gradient[0] = gradient[0] + (ka * (difference[0]))/distance
        # gradient[1] = gradient[1] + (ka * (difference[1]))/distance
        # gradient[2] = gradient[2] + (ka * (difference[2]))/distance

        # Calculate the gradient due to the influence of the trench
        last_point = self.previous_waypoint
        help_vector = transp_pot_coord - last_point
        transp_help_vector = help_vector.transpose()
        line_vector = goal_point - last_point
        # print(last_point)
        print(help_vector)
        print(line_vector)
        print(transp_help_vector)
        term_1 = line_vector[2] * transp_help_vector[1] - line_vector[1] * transp_help_vector[2]
        term_2 = line_vector[0] * transp_help_vector[2] - line_vector[2] * transp_help_vector[0]
        term_3 = line_vector[1] * transp_help_vector[0] - line_vector[0] * transp_help_vector[1]

        gradient[:, 0] = gradient[:, 0] + (
                -1 * line_vector[2] * term_2 * kb + line_vector[1] * term_3 * kb) / (np.linalg.norm(
            line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
        gradient[:, 1] = gradient[:, 1] + (
                line_vector[2] * term_1 * kb - line_vector[0] * term_3 * kb) / (np.linalg.norm(
            line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
        gradient[:, 2] = gradient[:, 2] + (
                -1 * line_vector[1] * term_1 * kb + line_vector[0] * term_2 * kb) / (np.linalg.norm(
            line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
        # gradient[0] = (kb*(transp_help_vector[2]*term_2+transp_help_vector[1]*term_3))/(np.linalg.norm(np.cross(help_vector, line_vector), axis=1)*np.linalg.norm(help_vector))
        # gradient[1] = (kb*(help_vector[2]*term_1-help_vector[0]*term_3))/(np.linalg.norm(np.cross(help_vector, line_vector), axis=1)*np.linalg.norm(help_vector))
        # gradient[2] = (kb*(help_vector[1]*term_1-help_vector[0]*term_2))/(np.linalg.norm(np.cross(help_vector, line_vector), axis=1)*np.linalg.norm(help_vector))

    # Calculate the gradient to ensure the minimum flight height
    z_distance = pot_coordinates[2] - limit
    mask = z_distance < limit
    temp_gradient = kd * - z_distance / np.abs(z_distance ** 3)
    temp_gradient[mask] = -10 ** 12
    gradient[:, 2] = gradient[:, 2] + temp_gradient

    # print(gradient)
    # Define the message
    grad_list = gradient[:, 0].astype(np.float32).tolist()
    grad_list.extend(gradient[:, 1].astype(np.float32).tolist())
    grad_list.extend(gradient[:, 2].astype(np.float32).tolist())
    print(grad_list)
    gradient_field = potential_field_msgResponse()
    gradient_field.resp.data = grad_list
    return gradient_field