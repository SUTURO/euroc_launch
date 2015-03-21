FROM suturo/euroc-suturo-base

MAINTAINER Moritz Horstmann <mhorst@tzi.de>

ADD repositories/euroc_perception/ ./src/euroc_perception
ADD repositories/euroc_manipulation/ ./src/euroc_manipulation
ADD repositories/euroc_planning/ ./src/euroc_planning
ADD repositories/euroc_msgs/ ./src/euroc_msgs
ADD repositories/euroc_launch/ ./src/euroc_launch
RUN sudo chown suturo:suturo -R ./src && PS1='> ' source ~/.bashrc && \
	catkin_make