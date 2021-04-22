- ROS2 Dashing Turtlebot3 image creation (from isrlab/ros2_dashing:full):
	- docker run -it isrlab/ros2_dashing:full
		- (follow ROS2 Turtlebot3 install when you are inside the container)
		- (When you complete the install, exit the container without removing it)

	- docker login

	- docker commit <ros2_container_id> isrlab/ros2_dashing:turtlebot3

	- docker push isrlab/ros2_dashing:turtlebot3


- ROS2 Turtlebot3 install (inside Docker container):
	- sudo apt update

	- Gazebo 9 installation:
		- sudo apt install gazebo9 libgazebo9-dev

		- sudo apt install ros-dashing-gazebo-ros-pkgs

	- sudo apt install ros-dashing-turtlebot3 ros-dashing-turtlebot3-bringup ros-dashing-turtlebot3-cartographer ros-dashing-turtlebot3-description ros-dashing-turtlebot3-example ros-dashing-turtlebot3-fake-node ros-dashing-turtlebot3-fake-node-dbgsym ros-dashing-turtlebot3-gazebo ros-dashing-turtlebot3-gazebo-dbgsym ros-dashing-turtlebot3-msgs ros-dashing-turtlebot3-msgs-dbgsym ros-dashing-turtlebot3-navigation2 ros-dashing-turtlebot3-node ros-dashing-turtlebot3-node-dbgsym ros-dashing-turtlebot3-simulations ros-dashing-turtlebot3-teleop
