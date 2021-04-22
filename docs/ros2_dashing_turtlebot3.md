###### ros2 dashing turtlebot3 image build (part 1)

```console
user@host:~$ docker run -it isrlab/ros2_dashing:full
```

###### ros2 turtlebot3 install (inside isrlab/ros2_dashing:full container) (part 1)

```console
user@dockerslave:~$ sudo apt update
```

###### gazebo9 install (inside isrlab/ros2_dashing:full container)

```console
user@dockerslave:~$ sudo apt install gazebo9 libgazebo9-dev
user@dockerslave:~$ sudo apt install ros-dashing-gazebo-ros-pkgs
```

###### ros2 turtlebot3 install (inside isrlab/ros2_dashing:full container) (part 2)

```console
user@dockerslave:~$ sudo apt install ros-dashing-turtlebot3 ros-dashing-turtlebot3-bringup ros-dashing-turtlebot3-cartographer ros-dashing-turtlebot3-description ros-dashing-turtlebot3-example ros-dashing-turtlebot3-fake-node ros-dashing-turtlebot3-fake-node-dbgsym ros-dashing-turtlebot3-gazebo ros-dashing-turtlebot3-gazebo-dbgsym ros-dashing-turtlebot3-msgs ros-dashing-turtlebot3-msgs-dbgsym ros-dashing-turtlebot3-navigation2 ros-dashing-turtlebot3-node ros-dashing-turtlebot3-node-dbgsym ros-dashing-turtlebot3-simulations ros-dashing-turtlebot3-teleop
user@dockerslave:~$ exit 
```

###### ros2 dashing turtlebot3 image build (part 2)

```console
user@host:~$ docker login
user@host:~$ docker commit <ros2_container_id> isrlab/ros2_dashing:turtlebot3
user@host:~$ docker push isrlab/ros2_dashing:turtlebot3
```
