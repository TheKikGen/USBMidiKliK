#!/bin/fish

########################################
# dependencies of this repository
########################################

set dependencies \
	lufa \
	midiXparser

# for every entry in $dependencies:
# 	name uri version [init-command]

set lufa \
	lufa \
	https://github.com/abcminiuser/lufa.git \
	5ba628d10b54d58d445896290ba9799bd76a73b3 \
	"make"

set midiXparser \
	midiXparser \
	https://github.com/TheKikGen/midiXparser.git \
	bf2fadf897fa1d5eabf004a8b383e3243caf3a2f
