###### gzweb + models image build (part 1)

```console
user@host:~$ docker run -it isrlab/ros2_dashing:turtlebot3
```

###### gzweb + models install (inside isrlab/ros2_dashing:turtlebot3 container)

```console
user@dockerslave:~$ source /usr/share/gazebo/setup.sh
user@dockerslave:~$ sudo apt update
user@dockerslave:~$ sudo apt install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
user@dockerslave:~$ curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
user@dockerslave:~$ source ~/.bashrc
user@dockerslave:~$ nvm install 6
user@dockerslave:~$ cd ~
user@dockerslave:~$ git clone https://github.com/osrf/gzweb
user@dockerslave:~$ cd ~/gzweb
user@dockerslave:~$ git checkout gzweb_1.4.1
user@dockerslave:~$ npm run deploy --- -m
user@dockerslave:~$ exit
```

###### gzweb + models image build (part 1)

```console
user@host:~$ docker login
user@host:~$ docker commit <ros2_container_id> isrlab/ros2_dashing:gzweb_m
user@host:~$ docker push isrlab/ros2_dashing:gzweb_m
```
