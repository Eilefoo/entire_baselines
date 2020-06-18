# reinforcement_learning_ws

Super-repository for reinforcement learning

## Usage

### Setup

```bash
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
git clone git@github.com:unr-arl/reinforcement_learning_ws.git
cd reinforcement_learning_ws
git submodule update --init --recursive
catkin config -DCMAKE_BUILD_TYPE=Release --blacklist rotors_hil_interface ctrl_planning
catkin build
source devel/setup.bash
```

As we haven't started working with the ctrl_planning yet, its best to blacklist it as well as it has several dependancies that we shall not be using.

### Run

```bash
roslaunch rmf rmf_sim.launch
```

## Details

- By default RMF is equipped with an OS1-64. You can change it to 0S0-128 inside 
src/rmf_sim/rotors/urdf/delta.xacro (comment/uncomment)\
Remember to change the static_transform_publisher inside rmf_sim.launch (comment/uncomment)

- All the models/worlds can be found here https://github.com/unr-arl/gazebo_resources.git

