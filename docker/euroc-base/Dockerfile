FROM ggrandes/ubuntu32:12.04

MAINTAINER Tom Gehrke <tgehrke@tzi.de>

ENV DEBIAN_FRONTEND noninteractive

# base tools and graphics drivers
RUN apt-get update && \
	apt-get install -y ca-certificates wget apt-utils mesa-utils xserver-xorg-video-all && \
	apt-get clean

# ros hydro + euroc deps
RUN	sh -c '. /etc/lsb-release && \
	echo "deb http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list' && \
	wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - && \
	apt-get update && \
	apt-get install -y libgts-0.7-5 ros-hydro-desktop-full ros-hydro-moveit-full libprotobuf7 libtbb2 libtar0 libcegui-mk2-0.7.5 python-wstool && \
	rm /bin/sh && ln -s /bin/bash /bin/sh && \
	apt-get clean
# Add user suturo
RUN adduser --disabled-password --shell /bin/bash --gecos '' suturo
WORKDIR /home/suturo
RUN mkdir download && cd download && \
	wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-simulator_1.0.27_i386.deb && \
	wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-scenes_1.0.26_i386.deb && \
	wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-interface_1.0.18_i386.deb && \
	dpkg -i *.deb && cd .. && rm -rf download
USER suturo
ENV ROS_WORKSPACE /home/suturo/catkin_ws
RUN echo "source /opt/ros/hydro/setup.bash" >> /home/suturo/.bashrc && \
    echo "source /opt/euroc_c2s1/ros/install/setup.bash" >> /home/suturo/.bashrc && \
    echo "source $ROS_WORKSPACE/devel/setup.bash" >> /home/suturo/.bashrc && \
    mkdir -p $ROS_WORKSPACE/src
WORKDIR $ROS_WORKSPACE
CMD /bin/bash