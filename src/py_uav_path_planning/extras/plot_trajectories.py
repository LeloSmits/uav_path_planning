import numpy as np
import matplotlib.pyplot as plt
import pickle
import matplotlib.gridspec

if False:
    pickle_file = "/home/daniel/path_planning_logs/z=0,5/3d_1,kc=15.p"
    # open pickle file
    file_open = open(pickle_file, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos = np.array(pos_data[:, 1:4])

    pickle_file_px4 = "/home/daniel/path_planning_logs/baumann_3d_1.p"
    # open PX4 file
    file_open = open(pickle_file_px4, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos_px4 = np.array(pos_data[:, 1:4])


    # Plot oben

    lim2 = [-5,5]
    lim3 = [0., 8.]

    gskw = dict(height_ratios=[np.diff(lim2)[0], np.diff(lim3)[0]])
    gs = matplotlib.gridspec.GridSpec(2, 1, **gskw)

    fig0 = plt.figure(figsize=(7, 4), dpi=320)
    ax0 = fig0.add_subplot(gs[0, 0], aspect="equal", adjustable='box')
    ax1 = fig0.add_subplot(gs[1, 0], aspect="equal", adjustable='box', sharex=ax0)

    circle1 = plt.Circle((12, -2), .5, color='black', fill=False, lw=2)
    ax0.vlines((5.5, 6.5), -3.5, 3.5)
    ax0.hlines((3.5, -3.5), 5.5, 6.5)
    ax0.add_patch(circle1)

    ax1.vlines((5.5, 6.5), 0,1)
    ax1.hlines((0, 1), 5.5, 6.5)
    ax1.vlines((11.5, 12.5), 0,6)
    ax1.hlines((0, 6), 11.5, 12.5)


    plot_apf, = ax0.plot(uav_pos[:, 0], uav_pos[:, 1], label="APF", color="green")
    plot_vfh, = ax0.plot(uav_pos_px4[:,0], uav_pos_px4[:,1], label="VFH", color="orange")
    ax1.plot(uav_pos[:, 0], uav_pos[:, 2], label="APF", color="green")
    ax1.plot(uav_pos_px4[:,0], uav_pos_px4[:,2], label="VFH", color="orange")
    marker_one = ax0.scatter(24, 0, marker='x', color='r', linewidth=5, s=100, zorder=10)
    marker_two = ax0.scatter(0, 0, marker='x', color='b', linewidth=5, s=100, zorder=10)
    ax1.scatter(24, 1, marker='x', color='r', linewidth=5, s=100, zorder=10)
    ax1.scatter(0, 1, marker='x', color='b', linewidth=5, s=100, zorder=10)
    ax0.set_ylabel('y')
    ax1.set_xlabel('x')
    ax1.set_ylabel('z')
    ax1.set_ylim(0, 8)
    ax1.set_xlim(-1,30)
    ax0.set_xlim(-1,30)
    ax0.set_ylim(-5,5)
    ax0.legend(handles=[marker_one, marker_two, plot_apf, plot_vfh], labels=['Goal', 'Start', 'APF-Path', '3DVFH*-Path'],
               title='Legend', loc='best', bbox_to_anchor=(1.35, 1.05))
    ax0.set_title('3D - Scenario 1 - Flown Path')
    plt.tight_layout()
    #plt.show()
    plt.savefig('3D - Scenario 1 - Flown Path.svg', bbox_inches='tight')


if False:
    #############################
    pickle_file = "/home/daniel/path_planning_logs/z=0,5/3d_2,kc=15.p"
    # open pickle file
    file_open = open(pickle_file, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos = np.array(pos_data[:, 1:4])

    pickle_file_px4 = "/home/daniel/path_planning_logs/baumann_3d_2.p"
    # open PX4 file
    file_open = open(pickle_file_px4, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos_px4 = np.array(pos_data[:, 1:4])

    # Plot oben
    lim2 = [-10, 6.]
    lim3 = [0., 8.]

    gskw = dict(height_ratios=[np.diff(lim2)[0], np.diff(lim3)[0]])
    gs = matplotlib.gridspec.GridSpec(2, 1, **gskw)

    fig0 = plt.figure(figsize=(7, 5), dpi=320)
    ax0 = fig0.add_subplot(gs[0, 0], aspect="equal", adjustable='box')
    ax1 = fig0.add_subplot(gs[1, 0], aspect="equal", adjustable='box', sharex=ax0)

    circle1 = plt.Circle((12, -2), .5, color='black', fill=False, lw=2)
    circle2 = plt.Circle((12, 0), .5, color='black', fill=False, lw=2)
    circle3 = plt.Circle((12, 2), .5, color='black', fill=False, lw=2)
    ax0.vlines((5.5, 6.5), -3.5, 3.5)
    ax0.hlines((3.5, -3.5), 5.5, 6.5)
    ax0.add_patch(circle1)
    ax0.add_patch(circle2)
    ax0.add_patch(circle3)

    ax1.vlines((5.5, 6.5), 0,1)
    ax1.hlines((0, 1), 5.5, 6.5)
    ax1.vlines((11.5, 12.5), 0,6)
    ax1.hlines((0, 6), 11.5, 12.5)


    plot_apf, = ax0.plot(uav_pos[:, 0], uav_pos[:, 1], label="APF", color="green")
    plot_vfh, = ax0.plot(uav_pos_px4[:,0], uav_pos_px4[:,1], label="VFH", color="orange")
    ax1.plot(uav_pos[:, 0], uav_pos[:, 2], label="APF", color="green")
    ax1.plot(uav_pos_px4[:,0], uav_pos_px4[:,2], label="VFH", color="orange")
    marker_one = ax0.scatter(24, 0, marker='x', color='r', linewidth=5, s=100, zorder=10)
    marker_two = ax0.scatter(0, 0, marker='x', color='b', linewidth=5, s=100, zorder=10)
    ax1.scatter(24, 1, marker='x', color='r', linewidth=5, s=100, zorder=10)
    ax1.scatter(0, 1, marker='x', color='b', linewidth=5, s=100, zorder=10)
    ax0.set_ylabel('y')
    ax1.set_xlabel('x')
    ax1.set_ylabel('z')
    ax1.set_ylim(0,8)
    ax1.set_xlim(-1,30)
    ax0.set_xlim(-1,30)
    ax0.set_ylim(-10, 6)
    ax0.legend(handles=[marker_one, marker_two, plot_apf, plot_vfh], labels=['Goal', 'Start', 'APF-Path', '3DVFH*-Path'],
               title='Legend', loc='best', bbox_to_anchor=(1.35, 1.))
    ax0.set_title('3D - Scenario 2 - Flown Path')
    plt.tight_layout()
    #plt.show()
    plt.savefig('3D - Scenario 2 - Flown Path.svg', bbox_inches='tight')


if True:
    #############################
    pickle_file = "/home/daniel/path_planning_logs/z=0,5/3d_3,kc=15.p"
    # open pickle file
    file_open = open(pickle_file, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos = np.array(pos_data[:, 1:4])

    pickle_file_px4 = "/home/daniel/path_planning_logs/baumann_3d_3.p"
    # open PX4 file
    file_open = open(pickle_file_px4, "rb")
    pickle_data = pickle.load(file_open)
    pos_data = pickle_data['position_data']
    uav_pos_px4 = np.array(pos_data[:, 1:4])

    # Plot oben

    lim2 = [-17.5, 10]
    lim3 = [0., 8.]

    gskw = dict(height_ratios=[np.diff(lim2)[0], np.diff(lim3)[0]])
    gs = matplotlib.gridspec.GridSpec(2, 1, **gskw)

    fig0 = plt.figure(figsize=(5, 5), dpi=320)
    ax0 = fig0.add_subplot(gs[0, 0], aspect="equal", adjustable='box')
    ax1 = fig0.add_subplot(gs[1, 0], aspect="equal", adjustable='box', sharex=ax0)

    plt.setp(ax0.get_xticklabels(), visible=False)

    # ax0.set_aspect('equal', 'box')
    # ax1.set_aspect('equal', 'box')

    circle1 = plt.Circle((12, -2), .5, color='black', fill=False, lw=2)
    circle2 = plt.Circle((12, 0), .5, color='black', fill=False, lw=2)
    circle3 = plt.Circle((12, 2), .5, color='black', fill=False, lw=2)
    ax0.vlines((5.5, 6.5), -3.5, 3.5)
    ax0.hlines((3.5, -3.5), 5.5, 6.5)
    ax0.add_patch(circle1)
    ax0.add_patch(circle2)
    ax0.add_patch(circle3)
    ax0.vlines((17.5, 18.5), -9.5, 9.5)
    ax0.hlines((-9.5, 9.5), 17.5, 18.5)
    ax0.hlines((-8.5, 8.5), 17.5, 18.5, linestyle="dotted")

    ax1.vlines((5.5, 6.5), 0, 1)
    ax1.hlines((0, 1), 5.5, 6.5)
    ax1.vlines((11.5, 12.5), 0, 6)
    ax1.hlines((0, 6), 11.5, 12.5)
    ax1.vlines((17.5, 18.5), 0, 6)
    ax1.hlines((0, 6), 17.5, 18.5)
    ax1.hlines(5, 17.5, 18.5, linestyle="dotted")

    plot_apf, = ax0.plot(uav_pos[:, 0], uav_pos[:, 1], label="APF", color="green")
    plot_vfh, = ax0.plot(uav_pos_px4[:, 0], uav_pos_px4[:, 1], label="VFH", color="orange")
    ax1.plot(uav_pos[:, 0], uav_pos[:, 2], label="APF", color="green")
    ax1.plot(uav_pos_px4[:, 0], uav_pos_px4[:, 2], label="VFH", color="orange")
    marker_one = ax0.scatter(24, 0, marker='x', color='r', linewidth=5, s=100, zorder=10)
    marker_two = ax0.scatter(0, 0, marker='x', color='b', linewidth=5, s=100, zorder=10)
    ax1.scatter(24, 1, marker='x', color='r', linewidth=5, s=100, zorder=10)
    ax1.scatter(0, 1, marker='x', color='b', linewidth=5, s=100, zorder=10)
    # ax0.set_ylabel('y')
    # ax1.set_xlabel('x')
    # ax1.set_ylabel('z')
    ax1.set_ylim(0, 8)
    ax0.set_xlim(-1, 30)
    ax1.set_xlim(-1, 30)
    ax0.set_ylim(-17.5, 10)
    ax0.legend(handles=[marker_two,marker_one, plot_apf, plot_vfh], labels=['Start', 'Ziel', 'APF', '3DVFH*'],
               loc='lower left', fontsize=10) #, bbox_to_anchor=(1.4, 1.02))
    # ax0.set_title('3D - Scenario 3 - Flown Path')

    plt.tight_layout()
    fig0.savefig("kolloq_traj.svg")
    plt.show()
    #plt.savefig('3D - Scenario 3 - Flown Path.svg', bbox_inches='tight')
