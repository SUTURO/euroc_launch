#!/bin/bash
if [ "$PS1" == "> " ]; then
	exit # Exit when inside docker build environment.
fi

# Persist function usage: persist <file_or_dir_to_persist> [create symlink if file does not exist (true/false)] [Execute as root (true/false)]
function persist() {
	if [ "$3" == "true" ]; then
		PREFIX="sudo "
	fi
	if [ -h "$1" ]; then
		return 0
	fi
	# If file already exists in persistent container, delete and make symlink
	if [ -e "/persistent/$1" ]; then
		$PREFIX mkdir -p $(dirname $1)
		$PREFIX rm -rf $1
		$PREFIX ln -s /persistent/$1 $1
		return 0
	fi
	# If file to persist exists, move to persistent container and symlink
	if [ -f "$1" ] || [ -d "$1" ]; then
		mkdir -p /persistent/$(dirname $1)
		$PREFIX mv $1 /persistent/$1
		$PREFIX ln -s /persistent/$1 $1
	fi
	# If the file does not exist and second arg is true, create a new folder in the persistent container and symlink it
	if [ ! -e "$1" ] && [ "$2" == "true" ]; then
		$PREFIX mkdir -p $(dirname $1)
		mkdir -p /persistent/$1
		$PREFIX ln -s /persistent/$1 $1
	fi
}
persist /home/suturo/.rviz true
persist /home/suturo/.cache true
persist /home/suturo/.java true
if [ -e "/home/suturo/custompersist.sh" ]; then
	source "/home/suturo/custompersist.sh"
fi
