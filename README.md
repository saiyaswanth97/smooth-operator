# smooth-operator

# 🤖 ROS TurtleBot3 MPC Controller

A Model Predictive Control (MPC) implementation for autonomous navigation with TurtleBot3 in Gazebo simulation.

---

## Table of Contents

- [System Requirements](#-system-requirements)
- [Dependencies](#-dependencies)
- [Installation](#-installation)
- [Project Structure](#-project-structure)
- [Quick Start](#-quick-start)
- [Usage Guide](#-usage-guide)
- [Troubleshooting](#-troubleshooting)

---

## System Requirements

This project has been tested and verified on:

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 20.04 LTS |
| **ROS** | Noetic |
| **CMake** | 3.16+ |
| **Robot** | TurtleBot3 (Burger/Waffle) |
| **Python** | 3.8+ |

---

## Dependencies

### Required Packages

```bash
# ROS Noetic (if not already installed)
sudo apt update
sudo apt install ros-noetic-desktop-full

# TurtleBot3 packages
sudo apt install ros-noetic-turtlebot3-*

# Python dependencies
pip3 install numpy matplotlib scipy
```

### Optional (MPC Planner only)

**OSQP-Eigen** - Required for the MPC controller optimization

```bash
# Install Eigen3
sudo apt install libeigen3-dev

# Install OSQP-Eigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone <repository-url>
cd ..
```

### 2. Build the Workspace

```bash
# Navigate to workspace root
cd ~/catkin_ws

# Build all packages
catkin_make

# Source the workspace
source devel/setup.bash
```

**Note**: Add the source command to your `~/.bashrc` for convenience:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## Project Structure

```
smooth-operator/
├── src/
│   ├── CMakeLists.txt                   # Top-level CMake config
│   │
│   ├── mpc_controller/                  # MPC Controller Package
│   │   ├── include/                     # Header files
│   │   └── src/
│   │       └── mpc_node.cpp             # MPC controller implementation
│   │
│   ├── simulator/                       # Simulator Package
│   │   ├── launch/
│   │   │   └── simulator.launch         # Main launch file
│   │   ├── msg/                         # Custom message definitions
│   │   ├── srv/                         # Custom service definitions
│   │   ├── rviz/
│   │   │   └── default.rviz             # RViz visualization config
│   │   ├── scripts/
│   │   │   └── vehicle_node.py          # Vehicle dynamics & control
│   │   └── run_gazebo.sh                # Gazebo startup script
│   │
│   └── trajectory/                      # Trajectory Planning
│       └── scripts/                     # Trajectory generation scripts
│
├── build/                               # Build artifacts (auto-generated)
├── devel/                               # Development space (auto-generated)
│
├── docs/
│   └── architecture/
│
├── trajectory_data/                     # Trajectory datasets
│   └── waypoints/
│
├── workflows/                           # CI/CD workflows
│   ├── cpp-linter.yml
│   └── ruff.yml
│
├── pyproject.toml                       # Python project configuration
├── LICENSE
└── README.md
```

---

## Quick Start

### Single Command Launch

For a complete setup in one command:

```bash
roscore &
./src/simulator/run_gazebo.sh
roslaunch simulator simulator.launch
```

This will start:
- Gazebo physics simulation
- TurtleBot3 model
- RViz visualization
- Vehicle node
- MPC controller

---

## Usage Guide

### Method 1: Step-by-Step Launch (Recommended for Development)

This method gives you better control and faster iteration during development.

#### Step 1: Start Gazebo (Keep Running)

```bash
./src/simulator/run_gazebo.sh
```

**💡 Pro Tip**: Keep this terminal running! Leaving Gazebo open significantly reduces startup time for subsequent runs. You don't need to restart Gazebo between experiments.

#### Step 2: Launch the Simulator Node

```bash
# In a new terminal
python3 src/simulator/scripts/vehicle_node.py
```

#### Step 3: Open RViz for Visualization

```bash
# In a new terminal
rviz -d src/simulator/rviz/default.rviz
```

#### Step 4: Run the MPC Controller

```bash
# In a new terminal
rosrun mpc_controller mpc_node
```

---


## Testing the Setup

After launching, verify everything is working:

### 1. Check Active Nodes

```bash
rosnode list
```

Expected output:
```
/gazebo
/vehicle_node
/mpc_controller
/rviz
```

### 2. Monitor Topics

```bash
# List all active topics
rostopic list

# Echo vehicle state
rostopic echo /vehicle_state

# Echo control commands
rostopic echo /cmd_vel
```

### 3. Visualize in RViz

You should see:
- TurtleBot3 model
- Planned path trajectory
- Current position and orientation
- Sensor data (if equipped)

<!--
---

## Troubleshooting

### Common Issues

#### Issue: `catkin_make` fails with CMake errors

**Solution**: Ensure you have the correct CMake version
```bash
cmake --version  # Should be 3.16+
```

#### Issue: Gazebo crashes or won't start

**Solution**: Clear Gazebo cache
```bash
rm -rf ~/.gazebo/
killall gzserver gzclient
```

#### Issue: TurtleBot3 model not found

**Solution**: Set the TurtleBot3 model environment variable
```bash
export TURTLEBOT3_MODEL=burger
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

#### Issue: OSQP-Eigen not found during compilation

**Solution**: Verify installation and update CMake cache
```bash
sudo ldconfig
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make
```

#### Issue: Python script can't find modules

**Solution**: Check your Python path
```bash
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src
``` 

--- 

## Advanced Configuration

### Adjusting MPC Parameters

Edit `src/mpc_controller/config/mpc_params.yaml` to tune:
- Prediction horizon
- Control weights
- Constraints
- Update rate

### Custom Path Planning

Modify spline generation in `utils/spline.py`:
```python
points = np.array([[0, 0], [1, 2], [2, 0], [3, 3]])
spline = Spline(points)
```

---

## Development Workflow

**Recommended workflow for fast iteration:**

1. Start Gazebo once: `./src/simulator/run_gazebo.sh`
2. Keep it running in the background
3. Make code changes
4. Rebuild only if needed: `catkin_make`
5. Restart only the specific node you modified
6. Test and repeat

This approach saves significant time by avoiding Gazebo restarts!

---


## 📄 License

[Your License Here]

## 🤝 Contributing

[Contributing Guidelines]

## 📧 Contact

saiyaswanth97@gmail.com
-->
---

**Happy Navigating! 🚀**