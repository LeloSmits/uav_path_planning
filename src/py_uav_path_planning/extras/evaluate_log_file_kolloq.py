# coding=utf-8
import pickle
import numpy as np
import matplotlib.pyplot as plt



def evaluate(pickle_file, pickle_file_px4, pickle_file_2, pickle_file_px4_2, names, figtitle):
    """Plots the log files created by the path_logger_node.
    :arg pickle_file: string of the path to the first log file
    :arg pickle_file_px4: string of the the path to the second log file, optional, defaults to None
    :arg names: tuple of strings for the two log files. Used for the legend in the plots. Optional, defaults to ("Log 1", "Log 2")
    :arg figtitle: title for the entire plot. Optional, defaults to None"""

    def calc_point_projected_on_line(vector_x, vector_a, vector_b):
        """Returns the closest point on the vector through a and b when projecting x on that line.
        https://stackoverflow.com/questions/5227373/minimal-perpendicular-vector-between-a-point-and-a-line"""
        direction = (vector_b - vector_a) / np.linalg.norm((vector_b - vector_a))
        vector_proj = vector_a + ((vector_x - vector_a).dot(direction)) * direction

        return vector_proj

    # Setup the figure
    figsize = (6, 3)
    fig0, (ax1) = plt.subplots(1, 3, figsize=figsize, sharex="col")

    # Open the first log file
    # file_open = open(pickle_file, "rb")
    # pickle_data = pickle.load(file_open)
    # pos_data = pickle_data['position_data']
    # time_total_1 = pos_data[-1, 0] - pos_data[0, 0]
    #
    # # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
    # distance_from_path = np.zeros(pos_data.shape[0])
    # uav_pos = np.array(pos_data[:, 1:4])
    # uav_vel = np.array(pos_data[:, 10:13])
    # wp_next = np.array(pos_data[:, 4:7])
    # wp_prev = np.array(pos_data[:, 7:10])
    #
    # # Get the current distance from the path for every time step
    # for row_i in range(len(pos_data)):
    #     point_on_path = calc_point_projected_on_line(uav_pos[row_i], wp_prev[row_i], wp_next[row_i])[:2]
    #     distance_from_path[row_i] = np.linalg.norm(uav_pos[row_i][:2] - point_on_path)
    #
    # # Get the cumulated distance from the path for every time step
    # integral_distance_from_path = np.zeros(pos_data.shape[0])
    # for row_i in range(1, len(pos_data)):
    #     integral_distance_from_path[row_i] = integral_distance_from_path[row_i - 1] \
    #                                          + distance_from_path[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])
    #
    # acceleration = np.zeros(uav_vel.shape[0])
    # for row_i in range(1, len(uav_vel)):
    #     acceleration[row_i] = np.linalg.norm(
    #         (uav_vel[row_i, :2] - uav_vel[row_i - 1, :2]) / (pos_data[row_i, 0] - pos_data[row_i - 1, 0]))
    #
    # integral_acceleration = np.zeros(acceleration.shape)
    # for row_i in range(1, len(uav_vel)):
    #     integral_acceleration[row_i] = integral_acceleration[row_i - 1] \
    #                                    + acceleration[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])
    #
    # # Plot the current and cumulated distance from the path
    # # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
    # ax0[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path,
    #                    label=names[0] + ": Momentane Abweichung vom Pfad")
    # ax00 = ax0[0].twinx()
    # ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
    #                   label=names[0] + ": Kumulierte Abweichung vom Pfad", linestyle="--")
    #
    # ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 10] ** 2 + pos_data[:, 11] ** 2),
    #             label=names[0] + ': XY-Geschwindigkeit', color="#1f77b4")
    #
    # ax0[2].plot(pos_data[:, 0] - pos_data[0, 0], acceleration, label=names[0] + ': Momentane Beschleunigung',
    #                    color="#1f77b4")
    # ax02 = ax0[2].twinx()
    # ax02.plot(pos_data[:, 0] - pos_data[0, 0], integral_acceleration,
    #                   label=names[0] + ': Kumulierte Beschleunigung', color="#1f77b4", linestyle="--")
    #
    #
    # file_open = open(pickle_file_px4, "rb")
    # pickle_data = pickle.load(file_open)
    # pos_data = pickle_data['position_data']
    #
    # time_total_2 = pos_data[-1, 0] - pos_data[0, 0]
    #
    # # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
    # distance_from_path = np.zeros(pos_data.shape[0])
    # uav_pos = np.array(pos_data[:, 1:4])
    # uav_vel = np.array(pos_data[:, 10:13])
    # wp_next = np.array(pos_data[:, 4:7])
    # wp_prev = np.array(pos_data[:, 7:10])
    #
    # # Get the current distance from the path for every time step
    # for row_i in range(len(pos_data)):
    #     point_on_path = calc_point_projected_on_line(uav_pos[row_i], wp_prev[row_i], wp_next[row_i])[:2]
    #     distance_from_path[row_i] = np.linalg.norm(uav_pos[row_i][:2] - point_on_path)
    #
    # # Get the cumulated distance from the path for every time step
    # integral_distance_from_path = np.zeros(pos_data.shape[0])
    # for row_i in range(1, len(pos_data)):
    #     integral_distance_from_path[row_i] = integral_distance_from_path[row_i - 1] \
    #                                          + distance_from_path[row_i] * (
    #                                                  pos_data[row_i, 0] - pos_data[row_i - 1, 0])
    # acceleration = np.zeros(uav_vel.shape[0])
    # for row_i in range(1, len(uav_vel)):
    #     acceleration[row_i] = np.linalg.norm(
    #         (uav_vel[row_i, :2] - uav_vel[row_i - 1, :2]) / (pos_data[row_i, 0] - pos_data[row_i - 1, 0]))
    #
    # integral_acceleration = np.zeros(acceleration.shape)
    # for row_i in range(1, len(uav_vel)):
    #     integral_acceleration[row_i] = integral_acceleration[row_i - 1] \
    #                                    + acceleration[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])
    #
    # # Plot the current and cumulated distance from the path
    # # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
    # ax0[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path,
    #                      label=names[1] + ": Momentane Abweichung vom Pfad")
    # ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
    #                     label=names[1] + ": Kumulierte Abweichung vom Pfad", linestyle="--")
    # ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 10] ** 2 + pos_data[:, 11] ** 2),
    #             label=names[1] + ': XY-Geschwindigkeit', color="#ff7f0e")
    #
    # ax0[2].plot(pos_data[:, 0] - pos_data[0, 0], acceleration,
    #                      label=names[1] + ': Momentane Beschleunigung', color="#ff7f0e")
    # ax02.plot(pos_data[:, 0] - pos_data[0, 0], integral_acceleration,
    #                     label=names[1] + ': Kumulierte Beschleunigung', color="#ff7f0e", linestyle="--")
    #
    # ax0[0].set_ylim(0)
    # ax00.set_ylim(0)
    # ax0[0].set_title("Pfadabweichung", fontsize=10)
    # ax0[1].set_ylim(0)
    # ax0[1].set_title("Geschwindigkeit", fontsize=10)
    # ax0[2].set_ylim(0)
    # ax02.set_ylim(0)
    # ax0[2].set_title("Beschleunigung", fontsize=10)
    #
    # ax00.yaxis.set_visible(False)
    # ax02.yaxis.set_visible(False)

##################################################################################################
    # Open the first log file
    file_open = open(pickle_file_2, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    time_total_3 = pos_data[-1, 0] - pos_data[0, 0]

    # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
    distance_from_path = np.zeros(pos_data.shape[0])
    uav_pos = np.array(pos_data[:, 1:4])
    uav_vel = np.array(pos_data[:, 10:13])
    wp_next = np.array(pos_data[:, 4:7])
    wp_prev = np.array(pos_data[:, 7:10])

    # Get the current distance from the path for every time step
    for row_i in range(len(pos_data)):
        point_on_path = calc_point_projected_on_line(uav_pos[row_i], wp_prev[row_i], wp_next[row_i])
        distance_from_path[row_i] = np.linalg.norm(uav_pos[row_i] - point_on_path)

    # Get the cumulated distance from the path for every time step
    integral_distance_from_path = np.zeros(pos_data.shape[0])
    for row_i in range(1, len(pos_data)):
        integral_distance_from_path[row_i] = integral_distance_from_path[row_i - 1] \
                                             + distance_from_path[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])

    acceleration = np.zeros(uav_vel.shape[0])

    for row_i in range(1, len(uav_vel)):
        acceleration[row_i] = np.linalg.norm(
            (uav_vel[row_i] - uav_vel[row_i - 1]) / (pos_data[row_i, 0] - pos_data[row_i - 1, 0]))

    integral_acceleration = np.zeros(acceleration.shape)
    for row_i in range(1, len(uav_vel)):
        integral_acceleration[row_i] = integral_acceleration[row_i - 1] \
                                       + acceleration[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])

    # Plot the current and cumulated distance from the path
    # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
    ax1[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path,
                       label=names[0] + ": Momentane Abweichung vom Pfad")
    ax00 = ax1[0].twinx()
    ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
                      label=names[0] + ": Kumulierte Abweichung vom Pfad", linestyle="--")


    ax1[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 10] ** 2 + pos_data[:, 11] ** 2 + pos_data[:, 12] ** 2),
                label=names[0] + ': XYZ-Geschwindigkeit', color="#1f77b4")


    ax1[2].plot(pos_data[:, 0] - pos_data[0, 0], acceleration, label=names[0] + ': Momentane Beschleunigung',
                       color="#1f77b4")
    ax02 = ax1[2].twinx()
    ax02.plot(pos_data[:, 0] - pos_data[0, 0], integral_acceleration,
                      label=names[0] + ': Kumulierte Beschleunigung', color="#1f77b4", linestyle="--")


    # Open the second log file
    file_open = open(pickle_file_px4_2, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    meta_data = pickle_data['meta_data']

    # Print the total time for
    time_total_4 = pos_data[-1, 0] - pos_data[0, 0]

    # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
    distance_from_path = np.zeros(pos_data.shape[0])
    uav_pos = np.array(pos_data[:, 1:4])
    uav_vel = np.array(pos_data[:, 10:13])
    wp_next = np.array(pos_data[:, 4:7])
    wp_prev = np.array(pos_data[:, 7:10])

    # Get the current distance from the path for every time step
    for row_i in range(len(pos_data)):
        point_on_path = calc_point_projected_on_line(uav_pos[row_i], wp_prev[row_i], wp_next[row_i])
        distance_from_path[row_i] = np.linalg.norm(uav_pos[row_i] - point_on_path)

    # Get the cumulated distance from the path for every time step
    integral_distance_from_path = np.zeros(pos_data.shape[0])
    for row_i in range(1, len(pos_data)):
        integral_distance_from_path[row_i] = integral_distance_from_path[row_i - 1] \
                                             + distance_from_path[row_i] * (
                                                     pos_data[row_i, 0] - pos_data[row_i - 1, 0])
    acceleration = np.zeros(uav_vel.shape[0])
    for row_i in range(1, len(uav_vel)):
        acceleration[row_i] = np.linalg.norm(
            (uav_vel[row_i] - uav_vel[row_i - 1]) / (pos_data[row_i, 0] - pos_data[row_i - 1, 0]))

    integral_acceleration = np.zeros(acceleration.shape)
    for row_i in range(1, len(uav_vel)):
        integral_acceleration[row_i] = integral_acceleration[row_i - 1] \
                                       + acceleration[row_i] * (pos_data[row_i, 0] - pos_data[row_i - 1, 0])

    # Plot the current and cumulated distance from the path
    # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
    ax1[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path,
                         label=names[1] + ": Momentane Abweichung vom Pfad")
    ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
                        label=names[1] + ": Kumulierte Abweichung vom Pfad", linestyle="--")
    ax1[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 10] ** 2 + pos_data[:, 11] ** 2 + pos_data[:, 12] ** 2),
                label=names[1] + ': XYZ-Geschwindigkeit', color="#ff7f0e")


    ax1[2].plot(pos_data[:, 0] - pos_data[0, 0], acceleration,
                         label=names[1] + ': Momentane Beschleunigung', color="#ff7f0e")
    ax02.plot(pos_data[:, 0] - pos_data[0, 0], integral_acceleration,
                        label=names[1] + ': Kumulierte Beschleunigung', color="#ff7f0e", linestyle="--")

    ax00.yaxis.set_visible(False)
    ax02.yaxis.set_visible(False)

    # Set the
    # for ax_i in ax0:
    #     ax_i.grid()
    for ax_i in ax1:
        ax_i.grid()
        ax_i.set_xlim(0, max(time_total_3, time_total_4))  # , pos_data[-1, 0] - pos_data[0, 0])
    ax1[0].set_ylim(0)
    ax00.set_ylim(0)
    #ax0[0].set_title("Pfadabweichung", fontsize=8)

    #ax0[1].set_title("XY-Geschwindigkeit", fontsize=8)

    ax1[1].set_ylim(0)

    #ax0[2].set_title("Beschleunigung", fontsize=8)
    ax1[2].set_ylim(0)
    # ax02.set_ylim(0)
    # ax1[0].set_title("Pfadabweichung", fontsize=10)
    # ax1[1].set_title("Geschwindigkeit", fontsize=10)
    # ax1[2].set_title("Beschleunigung", fontsize=10)
    plt.tight_layout()

    plt.show()
    #fig0.savefig("plots/" + figtitle + ".svg")
    fig0.savefig("kolloq_graph.svg")

if __name__ == '__main__':
    evaluate("/home/daniel/path_planning_logs/Leo/path_log_scenario_3_ours.p",
             "/home/daniel/path_planning_logs/Leo/path_log_px4_scenario_3.p",
             "/home/daniel/path_planning_logs/z=0,5/3d_3,kc=15.p", "/home/daniel/path_planning_logs/baumann_3d_3.p",
             names=("2D-APF", "2D-VFH", "3D-APF", "3D-VFH"), figtitle=None)
