# MPC Follower for TurtleBot3 in Gazebo

This ROS 2 package implements a **Model Predictive Control (MPC)** algorithm for a TurtleBot3 robot to follow a moving target while avoiding obstacles in a Gazebo simulation.

**Features**

- MPC-based target tracking
- Obstacle avoidance using soft constraints
- Gazebo integration with custom world and models
- ROS 2 launch file to start the entire simulation
- Easily extendable and modular

---

**Package Structure**

```text
src/
└── mpc_follower/
    ├── launch/                     # Launch files
    │   └── launch_all.py
    ├── mpc_follower/             
    │   └── mpc_follower_node.py    # Main MPC node
    │   └── mpc_solver.py           # MPC optimization logic
    ├── setup.py
    ├── setup.cfg
    └── README.md
└── my_world/
       ├── models/                  # Gazebo models (target, obstacles)
```

---

**Dependencies**

Ensure the following are installed:

- ROS 2 Humble (or compatible distro)
- Gazebo (with gazebo_ros)
- casadi for nonlinear optimization
- bash
- pip install casadi

---

**Installation**

Clone the repo
```text
git clone https://github.com/HafXa-Xiddiqui/MPC-Nav.git
```
---

**Build**

```text
cd ~/MPC-Nav
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```
---

**Launch the full simulation**
```text
ros2 launch mpc_follower mpc_sim.launch.py
```
This will:

   - Launch Gazebo with an empty world
   - Spawn the target and obstacle models
   - Start the MPC controller node

---
**Demo**

![image](https://github.com/user-attachments/assets/c065e45d-7463-4275-b910-9ad49108ec7f)

---

**Parameters**

Tuneable parameters in the mpc_follower_node.py:
- N	Prediction horizon
- Ts	Sampling time (s)
- Q_pos	Position tracking weight
- R_v	Linear velocity weight
- R_omega	Angular velocity weight
- alpha_obs	Obstacle penalty weight
- d_goal	Desired distance behind target

---

**References**

 - CasADi documentation
 - TurtleBot3 documentation





