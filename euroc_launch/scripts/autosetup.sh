#!/bin/bash
echo
echo Installing EuRoC stack
echo 
ROOT=~
sudo apt-get update
sudo apt-get upgrade

# Simulator dependencies
sudo apt-get -y install wget libgts-0.7-5 ros-hydro-desktop-full ros-hydro-moveit-full libprotobuf7 libtbb2 libtar0 libcegui-mk2-0.7.5 python-wstool

# Perception dependencies
sudo apt-get -y install libpcl-1.7-all ros-hydro-pcl-ros ros-hydro-pcl-conversions gdb build-essential python-dev python-setuptools python-numpy python-scipy libatlas-dev libatlas3gf-base python-pip
/usr/bin/yes | pip install --user --install-option="--prefix=" -U scikit-learn

mkdir -p $ROOT
mkdir -p $ROOT/download
cd $ROOT/download
wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-interface_1.0.18_i386.deb
sudo dpkg -i euroc-c2s1-interface_1.0.18_i386.deb
if [ "$1" == "--with-server" ]
	then
	wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-scenes_1.0.25_i386.deb
	wget --user=euroc_qualif_c2 --password=4gWHsApa http://projects.laas.fr/euroc/euroc_qualif_c2/native/euroc-c2s1-simulator_1.0.27_i386.deb
	sudo dpkg -i euroc-c2s1-scenes_1.0.25_i386.deb euroc-c2s1-simulator_1.0.27_i386.deb
fi
tee $ROOT/.ssh/id_rsa <<KEYFILE
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAqnUQpq1aCcddfqhPg7dDWd8cI6uffLWKCCOz32vIecp3VmJn
7if7OfozcsYT3PqqPOlQYi8xdoQzjrfthSFbIFu5G9u7YPtZQv1rHDGIaSS4arB4
e3CNbHT+ltOZb+TwiZdaUTIF5RErhs1KtR1BKiPOlndHoU2GhWfcVqP6+oFf8oqB
dMIwrClJjiQ2WfNha5MAYsilUcTBcXgTtiZUWWbB4cQrP9cz/gCxl39pCkUaQjME
0PFfTczZJdF5wDh3GmpR73X0IXyvDmlz/EXXn9d+sBWqrx79Spy+AIzdr/JkW/gG
JJfiDXPlKmFpekABTVk8Sh9yzBNExRCFOWQBxwIDAQABAoIBAQCY7VAUCaCm90tl
L8GuzUNKpQVM51O+Ae0lokplHwEMa14njT/rBcm52URBK350mhsTbMsmW2rIBpFu
8IrTDvr/i+sGGFwDBV2u4UcwUywsELq3VT2ymWb4L/qc/JhDMCWxe62W/QcW3RGs
7g+hb+6fzOIyUi9cgv42P9kJjjLEfMf2a5t+ZZIo4RxYpUYbDPAIE5mzj9GbrsbQ
zJAbqSL69hGtRmfaSyDN4tokiCzuifX6AtgocnQ22h9/BQPYKrSgdxwcU4zZlJT1
SMldV/Mg0/9STs7aX5pTkXFI0MNvhUZERmmsoU3jwEQ3oloa+c/3O2SgjD9DDaao
pzzAw1/BAoGBAOCKoJ9kQGkFyQLFxFtCrVFERk4EYoH5jtSJqsTeccD5fkdWhJb8
ZhFVHvSQGcQqsTLNcB4yB47xCcjthUvAGbJhsDK4YSnGmyLmaRCQq7HgL3Z8Avvv
MfJYKaMY6DruIhu520FyY8eQ5i844gyua+RP8wAusW/osNsq9r4djR8hAoGBAMJW
qST0U+voYqX/CkRFxhmJR0gJGSRPGJumOI8/84XCZh9iLVOoAobhERwBLIFEk72k
CHjcREg0MLUsbkjtzzNePWDcwV9BcXy3uhqWLrqQUc0XuUgv3CnroMLrOMuzVWX8
O2uj06trySfQIFIpkvlN98eaorv4NJjVGgmIZovnAoGAVERSQV8rOP690DeOGD2u
s6fuxoqouWSBsMu1dcqRYD2BoZ4bhlpUp76NLnUB5nerJoiowfw+j+tvo6NX+fSu
8B3vNSmy9S2evPtBhMD2Xnvm9A1YRVogIXXqslW/mZYS4NrgKPCX96YubJxpYR74
SKekZ3YeiwQFk0rNbM3ET2ECgYBUjB9KVIGuQIuqbCqrSS/GLFM2o76sbTzneolg
yRHJP6nuPWg63P08cO5goKIcGxmrh5GoAywxvYLzT0EruUpLrtRFL8VxC6ez+rqc
lnn98IumDowtO6UtF7X5Fxy+gmbE19eCHOQZz+rTx1hIZo3qqGEEFeJ2pKU9WHjb
bNwwQQKBgCWPl6vf3VNdtNzy4dJQenyTP1oWtqwvdMFgbarXODXaTU175a7EbunQ
eRjonEvBszt1vdgGXTAYatXja2r/SSbFSh1EQ5smdHwM6k1VrKqilDJIBRD/RNr+
GQmxCIy0eCpznuHE9AKYmCFEuk7ty5HMGKxCTUj736AnYt86zvHL
-----END RSA PRIVATE KEY-----
KEYFILE
chmod -f 600 $ROOT/.ssh/id_rsa
tee $ROOT/.ssh/id_rsa.pub <<PUBFILE
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQCqdRCmrVoJx11+qE+Dt0NZ3xwjq598tYoII7Pfa8h5yndWYmfuJ/s5+jNyxhPc+qo86VBiLzF2hDOOt+2FIVsgW7kb27tg+1lC/WscMYhpJLhqsHh7cI1sdP6W05lv5PCJl1pRMgXlESuGzUq1HUEqI86Wd0ehTYaFZ9xWo/r6gV/yioF0wjCsKUmOJDZZ82FrkwBiyKVRxMFxeBO2JlRZZsHhxCs/1zP+ALGXf2kKRRpCMwTQ8V9NzNkl0XnAOHcaalHvdfQhfK8OaXP8Rdef136wFaqvHv1KnL4AjN2v8mRb+AYkl+INc+UqYWl6QAFNWTxKH3LME0TFEIU5ZAHH euroc@euroc01
PUBFILE
chmod -f 644 $ROOT/.ssh/id_rsa.pub

source /opt/ros/hydro/setup.bash
source /opt/euroc_c2s1/ros/install/setup.bash
mkdir -p $ROOT/euroc_ws/src
cd $ROOT/euroc_ws
wstool init
cd src
wstool set euroc_perception --git git@git.ai.uni-bremen.de:euroc_perception -y -v devel
wstool set euroc_manipulation --git git@git.ai.uni-bremen.de:euroc_manipulation -y -v devel
wstool set euroc_planning --git git@git.ai.uni-bremen.de:euroc_planning -y -v devel
wstool set euroc_msgs --git git@git.ai.uni-bremen.de:euroc_msgs -y -v devel
wstool set euroc_launch --git git@git.ai.uni-bremen.de:euroc_launch -y -v devel
wstool update
cd ..
catkin_make
source $ROOT/euroc_ws/devel/setup.bash
catkin_make
if [ "$1" == "--with-server" ]
	then
	rosrun euroc_launch install_integrationtest
fi