<p align="center">
<img src="./assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</p>

# **Learning Experience (LX): lx-estimation**

# About these activities

This is a learning experience about state estimation. 

In this learning experience, you will implement some toy examples of a Kalman filter 
and a particle filter, and then finally develop a histogram filter for the robot to use to 
estimate its position in the lane. 

The robot code of this learning experience runs on ROS 2 (Jazzy).

# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this README).


## 1. Make sure your exercise is up-to-date

Update your exercise definition and instructions,

    git pull upstream <your upstream branch>

**NOTE:** Example instructions to fork a repository and configure to pull from upstream can be found in the [duckietown-lx repository README](https://github.com/duckietown/duckietown-lx/blob/mooc2022/README.md).


## 2. Make sure your system is up-to-date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOT_NAME` (where `ROBOT_NAME` is the name of your Duckiebot chosen during the initialization procedure.)


## 3. Work on the exercise

### Launch the code editor

Open the code editor by running the following command,

```
dts code editor
```

Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar
of your browser to access the code editor. The first thing you will see in the code editor is
this same document, you can continue there.


### Walkthrough of notebooks

**NOTE**: You should be reading this from inside the code editor in your browser.

Inside the code editor, use the navigator sidebar on the left-hand side to navigate to the
`notebooks` directory and open the first notebook.

Follow the instructions on the notebook and work through the notebooks in sequence.


### Building your code

You can build your code with 

```
dts code build -R ROBOT_NAME
```

This will build a docker image with your code compiled inside - you should see your ROS 2 package
`histogram_lane_filter` get built (with `colcon`) during the process.


### Testing with Duckiematrix

In order to test your code in the Duckiematrix you will need a virtual robot. You can create one with the command:

```
dts duckiebot virtual create --type duckiebot --configuration DB21J [VBOT]
```

where `[VBOT]` can be anything you like (but remember it for later).

Then you can start your virtual robot with the command:

```
dts duckiebot virtual start [VBOT]
```

You should see it with a status `Booting` and finally `Ready` if you look at `dts fleet discover`: 

```
     | Hardware |   Type    | Model |  Status  | Hostname 
---  | -------- | --------- | ----- | -------- | ---------
[VBOT] |  virtual | duckiebot | DB21J |  Ready   | [VBOT].local
```

Now bring up the ROS 2 stack of the virtual robot (camera, wheel encoders, wheels driver). Virtual robots
still boot the ROS 1 stack by themselves; the ROS 2 stack is started explicitly for now:

```
dts stack up -R [VBOT] ros2/duckiebot
```

Now that your virtual robot is ready you can start the Duckiematrix. From this exercise directory do:

```
dts code start_matrix
```

You should see the Unity-based Duckiematrix simulator start up. 


### Testing on a real Duckiebot

After `dts duckiebot update ROBOT_NAME`, bring up the ROS 2 stack of the robot:

```
dts stack up -R ROBOT_NAME ros2/duckiebot
```

The histogram lane filter of this exercise detects the lane markings itself from the camera images, so the
lane-following demo stack (`dts stack up -R ROBOT_NAME ros2-core/default`) is not needed.


### 💻 Testing 


To test your code in the duckiematrix you can do:

```
dts code workbench -m -R [VIRTUAL_ROBOT_NAME]
```

and to test your code on your real Duckiebot you can do:

```
dts code workbench -R [ROBOT_NAME]
```

The workbench runs the node `histogram_lane_filter_node` (see
[packages/histogram_lane_filter](packages/histogram_lane_filter/README.md)) in the namespace of the robot.
It publishes the lane pose estimate on `histogram_lane_filter_node/lane_pose` and debug images of the
detected segments and of the belief histogram on `histogram_lane_filter_node/debug/.../compressed`.

To drive the robot around while the filter runs, use the keyboard controller in another terminal:

```
dts duckiebot keyboard_control [ROBOT_NAME]
```


### Watching the filter from the code editor

The last section of the [histogram filter notebook](notebooks/03-histogram-filter/histogram_filter.ipynb)
connects the notebook to the robot and shows the camera image, the detected segments, the belief histogram
and a live plot of `d` and `phi`; a parameter panel lets you tune the filter while it runs:

```python
from lx_ros2_tools.session import RosSession
from lx_ros2_tools.panel import ParamPanel, ActionButtons, ImageView, TopicPlot

ROBOT = "ROBOT_NAME"
ros = RosSession(ROBOT)
ImageView(ros, "histogram_lane_filter_node/debug/belief/compressed")
TopicPlot(ros, "histogram_lane_filter_node/lane_pose", "duckietown_msgs/msg/LanePose", ["d", "phi"])
ParamPanel(ros, "histogram_lane_filter_node", {"sigma_d_mask": (0, 5, 0.1), "sigma_phi_mask": (0, 5, 0.1)})
ActionButtons(ros, "histogram_lane_filter_node", ["reset"])
```

A terminal inside the code editor (Terminal > New Terminal) is already connected to the robot's ROS 2 graph
once the robot name is known (the `VEHICLE_NAME` environment variable, or after the `RosSession(ROBOT)` cell
of the notebook), so the usual ROS 2 tools work there, for example:

```
ros2 topic list
ros2 topic echo /ROBOT_NAME/histogram_lane_filter_node/lane_pose
ros2 node list
ros2 param list /ROBOT_NAME/histogram_lane_filter_node
ros2 param set /ROBOT_NAME/histogram_lane_filter_node sigma_d_mask 2.0
ros2 service call /ROBOT_NAME/histogram_lane_filter_node/reset std_srvs/srv/Trigger
```

Run `dt-ros2-connect ROBOT_NAME` in the terminal first so the `ros2` tools talk to your robot.


Now you can proceed to the [first notebook](notebooks/01-kalman-filter/kalman-filter.ipynb).
