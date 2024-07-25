# Robotics Lab Homework 4

### 1. Constructing a Gazebo World and Spawning the Mobile Robot

**a) Robot Spawning:**
- **Objective:** Launch Gazebo simulation and spawn the mobile robot in the `rl_racefield` world.
- **Pose Configuration:** Set robot pose to `x = -3`, `y = 5`, `yaw = -90°` with respect to the map frame. This is achieved by configuring the `spawn_frame_gazebo.launch` file with the necessary parameters.

**b) Obstacle Adjustment:**
- **Objective:** Move obstacle 9 in the `rl_racefield` world to a new position.
- **New Pose:** Set obstacle 9 to `x = -17`, `y = 9`, `z = 0.1`, `yaw = 3.14`. Adjustments were made to the `rl_race_field.world` file to update the obstacle’s pose.

**c) ArUco Marker Placement:**
- **Objective:** Place ArUco marker number 115 on obstacle 9.
- **Procedure:** Added the marker’s script and texture in the `materials` directory within the obstacle 9 description folder. Configured the SDF file to reference the marker and ensure its visibility to the robot’s camera.

<div align="center">
	<img src="https://github.com/saviodp7/RoboticsLabHW4/blob/main/images/world.png">
</div>

### 2. Setting Up Static TF Goals and Navigation

**a) Static TF Goals:**
- **Objective:** Place four static TF frames as navigation goals.
- **Goal Poses:**
  - Goal 1: `x = -10`, `y = 3`, `yaw = 0°`
  - Goal 2: `x = -15`, `y = 7`, `yaw = 30°`
  - Goal 3: `x = -6`, `y = 8`, `yaw = 180°`
  - Goal 4: `x = -17.5`, `y = 3`, `yaw = 75°`
- **Implementation:** Modified the `spawn_fra2mo_gazebo.launch` file to include these static TF frames.

**b) TF Listener Implementation:**
- **Objective:** Implement TF listeners to get and print goal poses.
- **Implementation:** Created `four_goals.cpp` to use `TransformListener` to retrieve and debug goal poses. The code listens to transformations and prints the position and orientation of each goal to the terminal.

**c) Goal Navigation with Move Base:**
- **Objective:** Command the robot to navigate through goals in a specified order using `move_base`.
- **Order:** Navigate through goals in the order: Goal 3 → Goal 4 → Goal 2 → Goal 1. Used `MoveBaseClient` to send goals and record the robot’s trajectory. Plotted the trajectory based on recorded data.

<div align="center">
	<img src="https://github.com/saviodp7/RoboticsLabHW4/blob/main/images/rviz.png">
</div>

### 3. Mapping and Tuning Navigation Parameters

**a) Additional Goals for Mapping:**
- **Objective:** Add goals to ensure a complete map of the environment.
- **Procedure:** Added three additional goals to strategically explore the environment. Updated the map to reflect these additions.

**b) Tuning Navigation Parameters:**
- **Objective:** Test different planner and `move_base` configurations to evaluate robot trajectories.
- **Configurations Tested:**
  1. **Race Mode:** Increased velocity and acceleration limits for agile performance but observed sudden decelerations near obstacles.
  2. **Obstacle Avoidance:** Adjusted minimum obstacle distance to improve avoidance but led to infeasible trajectories in narrow spaces.
  3. **Sharp Curves:** Reduced resolution and adjusted hysteresis to enable sharp turns and detailed costmap representation.
  4. **Minimal Look Ahead:** Decreased lookahead distance and increased acceleration, resulting in collisions due to insufficient stopping distance.

### 4. Vision-Based Navigation

**a) Running ArUco ROS Node:**
- **Objective:** Use the robot camera for ArUco marker detection.
- **Implementation:** Uncommented camera configuration in `fra2mo.xacro` and included necessary launch files for ArUco detection.

**b) 2D Navigation Task:**
- **Objective:** Navigate the robot to be near obstacle 9, detect the ArUco marker, and set a new goal.
- **Procedure:** Moved robot close to obstacle 9, detected the ArUco marker pose, and set the next goal one meter away from the marker’s pose using `MoveBaseClient`.

**c) Publishing ArUco Pose as TF:**
- **Objective:** Publish the ArUco marker’s pose as a TF frame.
- **Implementation:** Used `TransformBroadcaster` to create and send the TF frame with the marker’s pose, including rotation matrix to quaternion conversion for orientation.

<div align="center">
	<img src="https://github.com/saviodp7/RoboticsLabHW4/blob/main/images/mobile.gif">
</div>
