FROM ggrandes/ubuntu32:12.04

MAINTAINER Moritz Horstmann <mhorst@tzi.de>

# base tools and graphics drivers
RUN apt-get update && \
	apt-get install -y ca-certificates wget sudo zsh nano vim-nox tmux apt-utils mesa-utils xserver-xorg-video-all-lts-trusty && \
	apt-get clean

# ros hydro + euroc deps
RUN	sh -c '. /etc/lsb-release && \
	echo "deb http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list' && \
	wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - && \
	apt-get update && \
	apt-get install -y libgts-0.7-5 ros-hydro-desktop-full ros-hydro-moveit-full libprotobuf7 libtbb2 libtar0 libcegui-mk2-0.7.5 python-wstool && \
	rm /bin/sh && ln -s /bin/bash /bin/sh && \
	apt-get clean
# Add user suturo and generate locales
RUN adduser --disabled-password --shell /bin/bash --gecos '' suturo && \
	adduser suturo sudo && \
	adduser suturo video && \
	echo "%sudo ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
	locale-gen en_US.UTF-8
WORKDIR /home/suturo
# Install simulator
RUN mkdir download && cd download && \
	wget http://team.suturo.de:9419/files/euroc-c2s1-simulator_1.0.27_i386.deb && \
	wget http://team.suturo.de:9419/files/euroc-c2s1-scenes_1.0.25_i386.deb && \
	wget http://team.suturo.de:9419/files/euroc-c2s1-interface_1.0.18_i386.deb && \
	dpkg -i *.deb && cd .. && rm -rf download
USER suturo
# Set some environment variables
ENV ROS_WORKSPACE=/home/suturo/catkin_ws \
	QT_X11_NO_MITSHM=1 \
	HOME=/home/suturo \
	LANG=en_US.UTF-8 \
	LANGUAGE=en_US:en \ 
	LC_ALL=en_US.UTF-8
RUN echo "export JAVA_HOME=/usr/lib/jvm/java-6-openjdk-i386" >> /home/suturo/.bashrc && \
	echo "export ROS_HOSTNAME=localhost" >> /home/suturo/.bashrc && \
	echo "export ROS_IP=127.0.0.1" >> /home/suturo/.bashrc && \
	echo "export EUROC_HOST_IP_ADDRESS=\$ROS_IP" >> /home/suturo/.bashrc && \
	echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/suturo/.bashrc && \
	echo "source /opt/ros/hydro/setup.bash" >> /home/suturo/.bashrc && \
	echo "source /opt/euroc_c2s1/ros/install/setup.bash" >> /home/suturo/.bashrc && \
	echo "source $ROS_WORKSPACE/devel/setup.bash" >> /home/suturo/.bashrc && \
	echo "if [ -e /home/suturo/.custombashrc ] ; then source /home/suturo/.custombashrc; fi" >> /home/suturo/.bashrc && \
	echo "source /home/suturo/.bashrc" >> /home/suturo/.bash_profile && \
	mkdir -p $ROS_WORKSPACE/src && mkdir euroc_c2s1_logs
WORKDIR $ROS_WORKSPACE
CMD /bin/bash
