sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
ver="$(lsb_release -sr)"
if [ "$ver" = "20.04" ] ; then
  sudo apt install ros-noetic-desktop-full -y
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
if [ "$ver" = "18.04" ] ; then
  sudo apt install ros-melodic-desktop-full -y
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc

sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

