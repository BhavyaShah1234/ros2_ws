# ros2_ws

A ROS 2 Jazzy + Gazebo Harmonic workspace serving two roles: a shared underlay of robotic-arm and
autonomous-vehicle packages (Franka's own submodules below), and its own simulation projects built
around them.

The maze-solving project that used to live here (`maze_environment`, `maze_solver`) has moved to
its own workspace, [`maze_ws`](https://github.com/BhavyaShah1234/MazeSolverRobot), rebuilt with a
real sim/hardware boundary and overlaid on top of this workspace's `franka_description`/
`franka_ros2`/`libfranka` rather than vendoring its own copies -- see that repo for the current
version. This is also the intended pattern going forward: new arm/AV projects get their own
workspace and overlay this one for the shared hardware stack, instead of duplicating it.

## Projects

### Table tennis dual-robot simulation (`table_tennis_description`, `table_tennis_gazebo`)

Two namespaced Franka FR3 robots (`/red/`, `/green/`) with paddle end effectors face off across a table tennis table, with arm-mounted, overhead, and side RGBD cameras and a physics-based ball spawned via an action server. See [table_tennis_gazebo/README.md](src/table_tennis_gazebo/README.md) and [table_tennis_description/README.md](src/table_tennis_description/README.md) for setup details, controller verification, and troubleshooting; [FIXES_AND_USAGE.md](FIXES_AND_USAGE.md) covers the ball-spawner action API.

```bash
ros2 launch table_tennis_gazebo simulation.launch.py
```

### Franka arm support (submodules)

- [`franka_description`](https://github.com/frankaemika/franka_description) (`jazzy` branch) -- FR3 URDF/xacro and meshes
- [`franka_ros2`](https://github.com/frankaemika/franka_ros2) (`jazzy` branch) -- Franka's ROS 2 driver and control stack
- [`libfranka`](https://github.com/frankaemika/libfranka) (`main` branch) -- the underlying C++ control library

These are pinned as git submodules rather than committed directly, so upstream changes never need to be merged by hand -- see **Cloning** below.

## Cloning

The Franka packages are git submodules, so clone with `--recurse-submodules`:

```bash
git clone --recurse-submodules https://github.com/BhavyaShah1234/ros2_ws.git
```

If you already cloned without it:

```bash
git submodule update --init --recursive
```

## First-time setup (new machine)

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz
```

### 3. Install ros2_control and other dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-position-controllers \
  ros-jazzy-effort-controllers \
  ros-jazzy-velocity-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-moveit \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep
```

### 4. Clone and build

```bash
cd ~
git clone --recurse-submodules https://github.com/BhavyaShah1234/ros2_ws.git
cd ros2_ws

sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5. Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Add both lines to `~/.bashrc` to avoid re-sourcing every terminal. If you set `LD_LIBRARY_PATH` for anything else (CUDA, etc.), append to it rather than overwriting it -- overwriting drops the ROS libraries and controllers will fail to load.
