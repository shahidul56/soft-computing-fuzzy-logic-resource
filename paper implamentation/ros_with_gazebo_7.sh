
echo "installing all required package need to run simulation ......"

echo "\n"

echo " installing ros kinetics for ubuntu 16.0.4 ...." 

echo "\n" 

echo " install script by"

echo "\n"

echo "================== KUET CSE-MSC January batch 2019 ========================"

echo "\n"

echo "******************** CSE-6465: Soft Computing ****************************"


echo " Installing ROS KINETICS ........."

echo "\n"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full



sudo rosdep init
rosdep update


echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc


sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "\n"

echo "=====================  ros inatall finish ==========================================="

echo "\n"

echo "=====================  Installing trutlebot robot  ==========================================="

sudo apt-get install ros-kinetic-turtlebot
sudo apt-get install ros-kinetic-turtlebot-apps
sudo apt-get install ros-kinetic-turtlebot-interactions
sudo apt-get install ros-kinetic-turtlebot-simulator
sudo apt-get install ros-kinetic-kobuki-ftdi
sudo apt-get install ros-kinetic-ar-track-alvar-msgs
