# UAV Path Planning
## Table of contents

- [Quick start](#quick-start)
- [Status](#status)
- [What's included](#whats-included)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Contributing](#contributing)
- [Creators](#creators)
- [Thanks](#thanks)
- [Copyright and license](#copyright-and-license)


## Quick start

There are already a few example launch files in the folder ./launch. You can launch them with 
```text
roslaunch uav_path_planning path_planner_01.launch
```
or some other launch file of your choice.

## Status

Here goes all the budgets

## What's included

Some text

```text
uav_path_planning/
└── benchmarks/
    ├── scenarios/
    │   ├── 2d
    │   └── 3d
    └── classification_benchmark.csv        # Contains the danger index for different obstacle types; used by the benchmark launch files
└── launch/
    ├── benchmark_launch/
    │   └── local_planner_sitl_3cam.launch  # Start the PX4 3DVFH Planner, requires the PX4 avoidance package
    └── launch_files                        # Start various gazebo maps with our path planner
└── msg/
    ├── MultiArray.msg          # Message type used by the potential_field node
    ├── obstacleListMsg.msg     # Message type used by the obstacle_sensor and the obstacle_map nodes
    └── obstacleMsg.msg         # Message type used by the obstacle_sensor and the obstacle_map nodes
└── nodes/
    ├── global_path_node           
    ├── local_path_planner_node
    ├── obstacle_map_node
    ├── obstacle_sensor_node
    ├── path_logger_node
    └── setup_path_logger_for_px4_node
└── scripts/
    └── mavros_path_planning_test.py    # Just used for some testing
└── src/
    ├── folder3/
    │   ├── file1
    │   └── file2
    └── py_uav_path_planning/
        ├── file3
        └── file4
└── srv/
    ├── folder3/
    │   ├── file1
    │   └── file2
    └── folder4/
        ├── file3
        └── file4
```

## Bugs and feature requests

Have a bug or a feature request? Please first read the [issue guidelines](https://reponame/blob/master/CONTRIBUTING.md) and search for existing and closed issues. If your problem or idea is not addressed yet, [please open a new issue](https://reponame/issues/new).

## Contributing

Please read through our [contributing guidelines](https://reponame/blob/master/CONTRIBUTING.md). Included are directions for opening issues, coding standards, and notes on development.

Moreover, all HTML and CSS should conform to the [Code Guide](https://github.com/mdo/code-guide), maintained by [Main author](https://github.com/usernamemainauthor).

Editor preferences are available in the [editor config](https://reponame/blob/master/.editorconfig) for easy use in common text editors. Read more and download plugins at <https://editorconfig.org/>.

## Creators

**Creator 1**

- <https://github.com/usernamecreator1>

## Thanks

Some Text

## Copyright and license

Code and documentation copyright 2011-2018 the authors. Code released under the [MIT License](https://reponame/blob/master/LICENSE).

Enjoy :metal:
