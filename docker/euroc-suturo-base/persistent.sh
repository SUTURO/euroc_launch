#!/bin/bash
function persist() {
	if [ "$3" == "true" ]; then
		PREFIX="sudo "
	fi
	if [ -h "$1" ]; then
		return 0;
	fi
	if [ -f "$1" ] || [ -d "$1" ]; then
		mkdir -p /persistent/$(dirname $1)
		$PREFIX mv $1 /persistent/$1
		$PREFIX ln -s /persistent/$1 $1
	fi
	if [ ! -e "$1" ] && [ "$2" == "true" ]; then
		$PREFIX mkdir -p $(dirname $1)
		mkdir -p /persistent/$1
		$PREFIX ln -s /persistent/$1 $1
	fi
}
persist /data/db true true
persist /home/suturo/.rviz true
persist /home/suturo/.cache true
persist /home/suturo/.java true
if [ -e "/home/suturo/custompersist.sh" ]; then
	source "/home/suturo/custompersist.sh"
fi
