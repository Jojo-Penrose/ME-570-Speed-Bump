Bagfile is in the 00_Workspace/foxglove/recordings. Download speedbump_hit and upload that folder into bump_controller/bagfiles/

To run the bagfile, use:
```
ros2 bag play -l ~/ros2_ws/src/bump_controller/bagfiles/speedbump_hit/speedbump_hit_0.mcap
```

To build the package:
```
cd ~/ros2_ws/src
colcon build --symlink-install
source install/setup.bash
```

To run the bump node:
```
cd ~/ros2_ws/src
ros2 run bump_controller bump_node
```

To install keras:
'''
python3 -m pip install 'keras'
'''
