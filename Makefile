.POSIX:

MAKEFLAGS+=s

OS=$(shell uname -s)

ifndef SUBSCRIBER
	SUBSCRIBER=./src/subscriber
endif
ifndef PUBLISHER
	PUBLISHER=./src/publisher
endif

ifndef SUBSCRIBER_IMAGE
	SUBSCRIBER_IMAGE=virtual_robotics_subscriber
endif
ifndef PUBLISHER_IMAGE
	PUBLISHER_IMAGE=virtual_robotics_publisher
endif
ifndef SIMULATOR_IMAGE
	SIMULATOR_IMAGE=virtual_robotics_simulator
endif

.DEFAULT_GOAL := default

default: | rm up
rm: | down clean
v: | rm dev_v
r: | rm dev_r

up:
	command -V docker-compose || exit 1
	sudo docker-compose up

clean:
	if [ -d "$(PUBLISHER)/log" ]; then \
		sudo rm -rf "$(PUBLISHER)/log"; \
	fi
	echo "DIR $(PUBLISHER)/log removed"
	if [ -d "$(PUBLISHER)/install" ]; then \
		sudo rm -rf "$(PUBLISHER)/install"; \
	fi
	echo "DIR $(PUBLISHER)/install removed"
	if [ -d "$(PUBLISHER)/build" ]; then \
		sudo rm -rf "$(PUBLISHER)/build"; \
	fi
	echo "DIR $(PUBLISHER)/build removed"
	if [ -d "$(SUBSCRIBER)/log" ]; then \
		sudo rm -rf "$(SUBSCRIBER)/log"; \
	fi
	echo "DIR $(SUBSCRIBER)/log removed"
	if [ -d "$(SUBSCRIBER)/install" ]; then \
		sudo rm -rf "$(SUBSCRIBER)/install"; \
	fi
	echo "DIR $(SUBSCRIBER)/install removed"
	if [ -d "$(SUBSCRIBER)/build" ]; then \
		sudo rm -rf "$(SUBSCRIBER)/build"; \
	fi
	echo "DIR $(SUBSCRIBER)/build removed"

down:
	command -V docker >/dev/null || exit 1 
	echo "PRUNE all docker env objects"
	sudo docker system prune -f
	echo "RM $(SUBSCRIBER_IMAGE):latest docker image"
	sudo docker image ls | grep -i "$(SUBSCRIBER_IMAGE)" && \
		sudo docker rmi $(SUBSCRIBER_IMAGE):latest || \
		echo "image $(SUBSCRIBER_IMAGE):latest not found"
	echo "RM $(PUBLISHER_IMAGE):latest docker image"
	sudo docker image ls | grep -i "$(PUBLISHER_IMAGE)" && \
		sudo docker rmi $(PUBLISHER_IMAGE):latest || \
		echo "image $(PUBLISHER_IMAGE):latest not found"
	sudo docker image ls | grep -i "$(SIMULATOR_IMAGE)" && \
		sudo docker rmi $(SIMULATOR_IMAGE):latest || \
		echo "image $(SIMULATOR_IMAGE):latest not found"

dev_v: dev
	./dev v

dev_r: dev
	./dev r

.PHONY: default up clean down v r
