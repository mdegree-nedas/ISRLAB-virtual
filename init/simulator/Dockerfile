FROM isrlab/ros2_dashing:gzweb_m

MAINTAINER Valentino Di Giosaffatte valentino.digiosaffatte@student.univaq.it
MAINTAINER Riccardo Armando Di Prinzio riccardoarmando.diprinzio@student.univaq.it

EXPOSE 8080
EXPOSE 7681

SHELL ["/bin/bash", "-l", "-i", "-c"]

CMD source /usr/share/gazebo/setup.sh && \
        export GAZEBO_MASTER_URI=http://subscriber:11345 && \
	echo $GAZEBO_MASTER_URI && \
	cd /root/gzweb && \
	sleep 10 && \
	npm start
