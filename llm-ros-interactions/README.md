# LLM & Robotic Interaction Project
	- Have python and the python google-generativeai lib installed on your machine/construct

## Using "The Construct" Free-tier Workspace (Recommended)
	- Copy ros2_ws folder into your IDE
	- cd into the folder
	- Run:
		cd src
		ros2 pkg create --build-type ament_python gemini_ros2_interface --dependencies rclpy std_msgs
		cd ..
		colcon build --packages-select gemini_ros2_interface
		source install/setup.bash
	- open up two different terminals
	- In Terminal 1:
		- source ~/ros2_ws/install/setup.bash
		- ros2 run gemini_ros2_interface command_input_node
	- In Terminal 2:
		- source ~/ros2_ws/install/setup.bash
		- ros2 run gemini_ros2_interface gemini_processor_node



## Local System Setup (Not Recommended):
	- I'm working on an AWS EC2 using Ubuntu 22.04
	- Have cmake installed (sudo apt install cmake)

## ROS2 Install:

### set up your system, ensure utf8
	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

### install ubuntu universe, has some ros dependencies
	sudo apt install software-properties-common
	sudo add-apt-repository universe

### install curl if you don't have it, then add ROS2 GPG key with apt 
	sudo apt update && sudo apt install curl -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

### add repo to src list
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

### install common dependencies for ubuntu
	sudo apt update && sudo apt install -y \
	  python3-flake8-docstrings \
	  python3-pip \
	  python3-pytest-cov \
	  ros-dev-tools

	sudo apt install -y \
	   python3-flake8-blind-except \
	   python3-flake8-builtins \
	   python3-flake8-class-newline \
	   python3-flake8-comprehensions \
	   python3-flake8-deprecated \
	   python3-flake8-import-order \
	   python3-flake8-quotes \
	   python3-pytest-repeat \
	   python3-pytest-rerunfailures

### get ros code
	mkdir -p ~/ros2_humble/src
	cd ~/ros2_humble
	vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

### you might have some issues with vcs install, I had to add it manually to my path, like this: 
	pip install --user vcstool
	echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
	source ~/.bashrc
	vcs --version

### install ROS2 dependencies 
	sudo apt upgrade
	sudo rosdep init
	rosdep update
	rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

### how to build the code in your workspace
	cd ~/ros2_humble/
	colcon build --symlink-install

### you might have similar issues with colcon install, run this:
	pip install --user colcon-common-extensions
	echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
	source ~/.bashrc

