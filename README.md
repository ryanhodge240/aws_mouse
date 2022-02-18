# aws_mouse
```
git clone https://github.com/lbaitemple/aws_mouse/ 
cd aws_mouse
chmod +x updateos.sh
./updateos.sh
```
If you see any lock error, please try
```
sudo rm -r /var/lib/dpkg/lock*
sudo dpkg --configure -a
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
### running the robot. [node_follow_wall2.py has the cmd_vel topic to move the robot]
##### open another terminal
```
rosrun maze_demo node_follow_wall2.py
```

### running deep learning inferencing
#### open another terminal to run the classifier using deep learning neural network
```
pip install torch torchvision  future
EXPORT VEHICLE_NAME=''
roslaunch img_recognition infer.launch 
```
