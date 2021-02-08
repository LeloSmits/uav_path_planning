import pickle
import numpy as np
import matplotlib.pyplot as plt


def evaluate(pickle_file, pickle_file_2=None, names=("Log 1", "Log 2"), figtitle=None):
    """Plots the log files created by the path_logger_node.
    :arg pickle_file: string of the path to the first log file
    :arg pickle_file_2: string of the the path to the second log file, optional, defaults to None
    :arg names: tuple of strings for the two log files. Used for the legend in the plots. Optional, defaults to ("Log 1", "Log 2")
    :arg figtitle: title for the entire plot. Optional, defaults to None"""

    def calc_point_projected_on_line(vector_x, vector_a, vector_b):
        """Returns the closest point on the vector through a and b when projecting x on that line.
        https://stackoverflow.com/questions/5227373/minimal-perpendicular-vector-between-a-point-and-a-line"""
        direction = (vector_b - vector_a) / np.linalg.norm((vector_b - vector_a))
        vector_proj = vector_a + ((vector_x - vector_a).dot(direction)) * direction

        return vector_proj

    # Setup the figure
    figsize = (8.27, 11.69)
    fig0, ax0 = plt.subplots(2, figsize=figsize)
    if figtitle is not None:
        fig0.suptitle(figtitle, fontsize=20)

    # Open the first log file
    file_open = open(pickle_file, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    meta_data = pickle_data['meta_data']
    time_total_1 = pos_data[-1, 0] - pos_data[0, 0]
    print("Time for " + names[0] + ": " + str(time_total_1))

    # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
    distance_from_path = np.zeros(pos_data.shape[0])
    uav_pos = np.array(pos_data[:, 1:4])
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

    # Plot the current and cumulated distance from the path
    # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
    lns0 = ax0[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path, label=names[0] + ": Momentane Abweichung vom Pfad")
    ax00 = ax0[0].twinx()
    lns00 = ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
                      label=names[0] + ": Kumulierte Abweichung vom Pfad", linestyle="--")
    lns = lns0 + lns00
    ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 11]**2 + pos_data[:, 12]**2), label=names[0] + ': XY-Geschwindigkeit')
    ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], pos_data[:, 13], label=names[0] + ': Z-Geschwindigkeit')

    # This assignment is needed to correctly limit the x-axis later on. If pickle_file_2 is not None, this value is later overwritten
    time_total_2 = time_total_1

    # If pickle_file_2 was supplied, repeat the plot steps
    if pickle_file_2 is not None:
        # Open the second log file
        file_open = open(pickle_file_2, "rb")
        pickle_data = pickle.load(file_open)
        pos_data = pickle_data['position_data']
        meta_data = pickle_data['meta_data']

        # Print the total time for
        time_total_2 = pos_data[-1, 0] - pos_data[0, 0]
        print("Time for " + names[1] + ": " + str(time_total_2))

        # Get the position of the UAV for every time step and the corresponding global waypoints at that time step
        distance_from_path = np.zeros(pos_data.shape[0])
        uav_pos = np.array(pos_data[:, 1:4])
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

        # Plot the current and cumulated distance from the path
        # The lns0 and lns00 variables are needed to show the legend in a twin-axis-plot
        lns000 = ax0[0].plot(pos_data[:, 0] - pos_data[0, 0], distance_from_path,
                           label=names[1] + ": Momentane Abweichung vom Pfad")
        lns0000 = ax00.plot(pos_data[:, 0] - pos_data[0, 0], integral_distance_from_path,
                          label=names[1] + ": Kumulierte Abweichung vom Pfad", linestyle="--")
        ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], np.sqrt(pos_data[:, 11] ** 2 + pos_data[:, 12] ** 2),
                    label=names[1] + ': XY-Geschwindigkeit')
        ax0[1].plot(pos_data[:, 0] - pos_data[0, 0], pos_data[:, 13], label=names[1] + ': Z-Geschwindigkeit')
        lns = lns0 + lns00 + lns000 + lns0000

    # Set the
    for ax_i in ax0:
        ax_i.grid()
        ax_i.set_xlim(0, max(time_total_1, time_total_2))  # , pos_data[-1, 0] - pos_data[0, 0])
    ax0[0].set_ylim(0)
    ax00.set_ylim(0)
    ax00.grid(linestyle="--")
    ax0[0].set_title("Abweichung vom Pfad")
    labels = [l.get_label() for l in lns]
    ax00.legend(lns, labels)

    ax0[1].set_title("Lineare Geschwindigkeit")
    ax0[1].legend()

    if figtitle is not None:
        plt.tight_layout(rect=[0, 0, 1, 0.95])
    else:
        plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    evaluate("/home/daniel/path_planning_logs/z=0,5 atol=0,5/3d_1.p", "/home/daniel/path_planning_logs/baumann_3d_1.p", ("APF", "VFH"), "Scenario 1")
    evaluate("/home/daniel/path_planning_logs/z=0,5/3d_2,kc=15.p", "/home/daniel/path_planning_logs/baumann_3d_2.p", ("APF", "VFH"), "Scenario 2")
    evaluate("/home/daniel/path_planning_logs/z=0,5/3d_3,kc=15.p", "/home/daniel/path_planning_logs/baumann_3d_3.p", ("APF", "VFH"), "Scenario 3")
