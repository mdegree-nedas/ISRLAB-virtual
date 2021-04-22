OS = $(shell uname -s)

SUBSCRIBER ?= ./src/subscriber
PUBLISHER ?= ./src/publisher
SUBSCRIBER_IMAGE ?= virtual_robotics_subscriber
PUBLISHER_IMAGE ?= virtual_robotics_publisher
SIMULATOR_IMAGE ?= virtual_robotics_simulator

# debug vars
# [!] set your own

DEBUG_SUBSCRIBER ?= subscriber
DEBUG_SIMULATOR ?= simulator
DEBUG_PUBLISHER ?= publisher

DEBUG_SUBSCRIBER_WORKSPACE ?= /src/subscriber
DEBUG_SIMULATOR_WORKSPACE ?= /src/simulator
DEBUG_PUBLISHER_WORKSPACE ?= /src/publisher

DEBUG_ROOT_WORKSPACE ?= /root/workspace

DEBUG_TURTLEBOT3_IMAGE ?= isrlab/ros2_dashing:turtlebot3
DEBUG_SIMULATOR_IMAGE ?= isrlab/ros2_dashing:gzweb_m

DEBUG_PORT ?= "8080:8080"

DEBUG_NETWORK ?= rosnet
DEBUG_SHELL ?= zsh

DEBUG_HOST_REPO ?= ${HOME}/code/intelligent_systems_and_robotics_lab-virtual-proj
DEBUG_TERMINAL ?= st
DEBUG_DOCKER_CMD ?= docker
DEBUG_SLEEP_TIME ?= 5
