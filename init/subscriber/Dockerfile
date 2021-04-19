FROM isrlab/ros2_dashing:turtlebot3

MAINTAINER Valentino Di Giosaffatte valentino.digiosaffatte@student.univaq.it
MAINTAINER Riccardo Armando Di Prinzio riccardoarmando.diprinzio@student.univaq.it

# server
# subscriber
# listener
# gzserver

SHELL ["/bin/bash", "-l", "-i", "-c"]

CMD cd /root/workspace && \
    . /usr/share/gazebo/setup.sh && \
    colcon build --symlink-install && \
    . /root/workspace/install/setup.bash && \
    ln -sf /root/workspace/src/subscriber/models/maze_1 /usr/share/gazebo-9/models/ && \
	export TURTLEBOT3_MODEL=waffle_pi && \
	ros2 launch subscriber empty_world.launch.py

	# rosdep update && \
	# rosdep install -i --from-path src --rosdistro dashing -y && \
	# colcon build && \
