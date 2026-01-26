# :construction: Scenic :construction:
Scenic is an online open-set scene graph constructor designed for high-altitude UAVs.

### Running Scenic With ROS 2
After building and running the docker image with `run.bash`, run the following commands:
```
cd clipper/build
cmake --build .
sudo cmake --install . --prefix /usr/local
cd
cd lightgluestick
pip3 install --break-system-pacakges -r requirements.txt
cd build
cmake --build .
sudo cmake --install . --prefix /usr/local
cd
cd scenic/build
cmake --build .
sudo cmake --install . --prefix /usr/local
cd
export LD_LIBRARY_PATH=/usr/local/lib:/home/$USER/.local/lib/python3.12/site-packages/torch/lib:$LD_LIBRARY_PATH
cd ws
colcon build --symlink-install
source install/setup.bash
ros2 launch scenic_ros scenic-node.launch.py
```
...and in 19 easy steps, you too can run Scenic! (Fix for this coming soon).
