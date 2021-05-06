.POSIX:

include config.mk

.DEFAULT_GOAL := help

.PHONY: all # ----- (combo)[ros] build all
.PHONY: rm # ----- (combo)[ros] rm all

all: rm up
rm: down clean

# --------------------------------------------------
# HELP

.SILENT:
.PHONY: help # [make] print this help text
help:
	@grep '^.PHONY: .* #' $(firstword $(MAKEFILE_LIST)) |\
		sed 's/\.PHONY: \(.*\) # \(.*\)/\1 # \2/' |\
		awk 'BEGIN {FS = "#"}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' 

# --------------------------------------------------
# ROS RULES

.SILENT:
.PHONY: up # [ros] docker-compose up
up:
	command -V docker-compose || exit 1
	sudo docker-compose up

.SILENT:
.PHONY: down # [ros] docker-compose env clean and system prune
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

.SILENT:
.PHONY: clean # [ros] ros packages env clean
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

.SILENT:
.PHONY: debug # [debug] start docker debug env (set your own variables in config.mk)
debug:
	$(DEBUG_DOCKER_CMD) container rm -f $(DEBUG_SUBSCRIBER) $(DEBUG_SIMULATOR) $(DEBUG_PUBLISHER)
	$(DEBUG_DOCKER_CMD) network rm $(DEBUG_NETWORK) || true
	$(DEBUG_DOCKER_CMD) network create $(DEBUG_NETWORK)
	# subscriber
	$(shell (shell &>/dev/null $(DEBUG_TERMINAL) -e $(DEBUG_SHELL) -c "sleep $(DEBUG_SLEEP_TIME); $(DEBUG_DOCKER_CMD) run -it --net $(DEBUG_NETWORK) --rm -v $(DEBUG_HOST_REPO)$(DEBUG_SUBSCRIBER_WORKSPACE):$(DEBUG_ROOT_WORKSPACE) --name $(DEBUG_SUBSCRIBER) --workdir $(DEBUG_ROOT_WORKSPACE) $(DEBUG_TURTLEBOT3_IMAGE)") &)
	# simulator
	$(shell (shell &>/dev/null $(DEBUG_TERMINAL) -e $(DEBUG_SHELL) -c "sleep $(DEBUG_SLEEP_TIME); $(DEBUG_DOCKER_CMD) run -it --net $(DEBUG_NETWORK) --rm -v $(DEBUG_HOST_REPO)$(DEBUG_SIMULATOR_WORKSPACE):$(DEBUG_ROOT_WORKSPACE) -p $(DEBUG_PORT) --name $(DEBUG_SIMULATOR) --workdir $(DEBUG_ROOT_WORKSPACE) $(DEBUG_SIMULATOR_IMAGE)") &)
	# publisher
	$(shell (shell &>/dev/null $(DEBUG_TERMINAL) -e $(DEBUG_SHELL) -c "sleep $(DEBUG_SLEEP_TIME); $(DEBUG_DOCKER_CMD) run -it --net $(DEBUG_NETWORK) --rm -v $(DEBUG_HOST_REPO)$(DEBUG_PUBLISHER_WORKSPACE):$(DEBUG_ROOT_WORKSPACE) --name $(DEBUG_PUBLISHER) --workdir $(DEBUG_ROOT_WORKSPACE) $(DEBUG_TURTLEBOT3_IMAGE)") &)
	# subscriber ip redirect
	echo $(DEBUG_SUBSCRIBER)"_ip="$(shell $(DEBUG_DOCKER_CMD) inspect -f '{{ .NetworkSettings.Networks.'$(DEBUG_NETWORK)'.IPAddress }}' $(DEBUG_SUBSCRIBER)) > ./src/$(DEBUG_SUBSCRIBER)/ip
	# simulator ip redirect
	echo $(DEBUG_SIMULATOR)"_ip="$(shell $(DEBUG_DOCKER_CMD) inspect -f '{{ .NetworkSettings.Networks.'$(DEBUG_NETWORK)'.IPAddress }}' $(DEBUG_SIMULATOR)) > ./src/$(DEBUG_SIMULATOR)/ip
	echo $(DEBUG_SUBSCRIBER)"_ip="$(shell $(DEBUG_DOCKER_CMD) inspect -f '{{ .NetworkSettings.Networks.'$(DEBUG_NETWORK)'.IPAddress }}' $(DEBUG_SUBSCRIBER)) >> ./src/$(DEBUG_SIMULATOR)/ip
	# publisher ip redirect
	echo $(DEBUG_PUBLISHER)"_ip="$(shell $(DEBUG_DOCKER_CMD) inspect -f '{{ .NetworkSettings.Networks.'$(DEBUG_NETWORK)'.IPAddress }}' $(DEBUG_PUBLISHER)) > ./src/$(DEBUG_PUBLISHER)/ip
