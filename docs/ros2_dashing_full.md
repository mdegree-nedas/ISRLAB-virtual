###### ros2 dashing full env image build (part 1)

```console
user@host:~$ docker pull ros:dashing
user@host:~$ docker run -it ros:dashing
```

###### ros2 full env install (inside ros:dashing container)

```console
user@dockerslave:~$ sudo apt update && sudo apt install locales
user@dockerslave:~$ sudo locale-gen en_US en_US.UTF-8
user@dockerslave:~$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
user@dockerslave:~$ export LANG=en_US.UTF-8
user@dockerslave:~$ sudo apt update && sudo apt install curl gnupg2 lsb-release
user@dockerslave:~$ sudo apt update
user@dockerslave:~$ sudo apt install ros-dashing-desktop
user@dockerslave:~$ source /opt/ros/dashing/setup.bash
user@dockerslave:~$ sudo apt install -y python3-pip
user@dockerslave:~$ pip3 install -U argcomplete
```

###### test (inside ros:dashing container)
```
user@dockerslave:~$ ros2 run demo_nodes_cpp talker &
user@dockerslave:~$ ros2 run demo_nodes_py listener
user@dockerslave:~$ exit
```

###### ros2 dashing full env image build (part 2)

```
user@host:~$ docker login
user@host:~$ docker commit <ros2_container_id> isrlab/ros2_dashing:full
user@host:~$ docker push isrlab/ros2_dashing:full
```
