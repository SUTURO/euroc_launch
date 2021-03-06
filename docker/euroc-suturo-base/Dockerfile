# Set up all dependencies used by SUTURO software stack.
FROM team.suturo.de:9420/suturo/euroc-base
MAINTAINER Moritz Horstmann <mhorst@tzi.de>
USER root

# Install perception, manipulation
RUN apt-get update && apt-get install -y libpcl-1.7-all ros-hydro-pcl-ros ros-hydro-pcl-conversions gdb build-essential python-dev python-setuptools python-numpy python-scipy libatlas-dev libatlas3gf-base python-pip ros-hydro-sklearn python-sklearn \
	libconfig++8-dev autoconf python-rrdtool ros-hydro-roslisp-repl && \
	apt-get clean && \
	pip install -U tabulate pymongo==2.8.1 && \
	rosdep init

# Install extrawurst
RUN apt-get update && apt-get install -y bash-completion command-not-found tig zsh net-tools nano vim-nox openjdk-7-jdk libjsoncpp-dev aptitude qtcreator && \
	update-alternatives --install "/usr/bin/java" "java" "/usr/lib/jvm/java-1.7.0-openjdk-i386/jre/bin/java" 1 && \
 	update-alternatives --install "/usr/bin/javac" "javac" "/usr/lib/jvm/java-1.7.0-openjdk-i386/bin/javac" 1 && \
 	update-alternatives --set java "/usr/lib/jvm/java-1.7.0-openjdk-i386/jre/bin/java" && \
 	update-alternatives --set javac "/usr/lib/jvm/java-1.7.0-openjdk-i386/bin/javac" && \
 	wget http://team.suturo.de:9419/files/robomongo-0.8.4-i386.deb && dpkg -i robomongo-0.8.4-i386.deb && rm robomongo-0.8.4-i386.deb && \
 	pip install pygazebo requests && \
 	echo "export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-i386" >> /home/suturo/.bashrc && \
 	echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/lib/jvm/java-7-openjdk-i386/jre/lib/i386:/usr/lib/jvm/java-7-openjdk-i386/jre/lib/i386/server:/usr/lib/swi-prolog/lib/i386" >> /home/suturo/.bashrc && \
	apt-get clean

USER suturo
# Setup all 3rd party ROS packages in separate workspace
WORKDIR /home/suturo/3rdparty_ws
ADD .rosinstall ./.rosinstall
RUN sudo chown suturo:suturo . -R && mkdir src && rosdep update && wstool update && source /opt/ros/hydro/setup.bash && \
	source /opt/euroc_c2s1/ros/install/setup.bash && export ROS_WORKSPACE=/home/suturo/3rdparty_ws && \
	rosdep install -y --ignore-src --from-paths src/stacks/ && sudo apt-get clean -y && \
	cd src && touch pr2_common/{pr2_machine,pr2_common,pr2_description,pr2_dashboard_aggregator}/CATKIN_IGNORE pr2_controllers/{pr2_calibration_controllers,pr2_controllers,joint_trajectory_action,pr2_mechanism_controllers,pr2_gripper_action,single_joint_position_action,robot_mechanism_controllers,ethercat_trigger_controllers,pr2_head_action}/CATKIN_IGNORE cram_bridge/cram_fccl/CATKIN_IGNORE && cd .. && \
	catkin_make --pkg=designator_integration_msgs && catkin_make

#Add own semrec config.cfg
ADD semrec/config.cfg /home/suturo/3rdparty_ws/src/semrec/

# Setup main workspace on top of 3rd party workspace
WORKDIR /home/suturo/catkin_ws
RUN sudo chown suturo:suturo -R /home/suturo/3rdparty_ws && \
    source /opt/ros/hydro/setup.bash && cd src && catkin_init_workspace && cd .. && source /home/suturo/3rdparty_ws/devel/setup.bash && \
    catkin_make

ADD persistent.sh /home/suturo/persistent.sh