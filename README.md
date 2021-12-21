# aws_mouse
```
git clone https://github.com/lbaitemple/aws_mouse/ 
cd aws_mouse
chmod +x updateos.sh
./updateos.sh
```

### compile ROS
```
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.sh
```

### open virtual desktop
```
export DISPLAY=:0
roslaunch maze_demo explore_world.launch
```
