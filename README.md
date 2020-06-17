# reinforcement_learning_ws

Super-repository for reinforcement learning

## Usage

```bash
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
git clone git@github.com:unr-arl/reinforcement_learning_ws.git
cd reinforcement_learning_ws
git submodule update --init --recursive
catkin config -DCMAKE_BUILD_TYPE=Release --blacklist rotors_hil_interface
catkin build
source devel/setup.bash
```
