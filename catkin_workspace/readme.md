

This project is about how to use python3 with Anaconda to communicate with ROS Melodic in Ubuntu 18.

Before building the project, do as the following:

1. cd catkin_workspace
2. catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so 	%%%%% the path depends
3. catkin config --install

After building this project, do as the following:

1. conda activate "your_virtualenv_name"
2. cd catkin_workspace/
3. source install/setup.bash --extend

Then you can use python3 to send/receive pics to/from ROS Melodic.(it helps when you trained your network with python3)
eg. you have predict.py in project "segmentation".

1. cd segmentation/src/scripts
2. python predict.py

That's all.
