#!/bin/bash

# Copyright (c) 2011 Autonomous Robots and Cognitive Systems Laboratory, Universidad de Costa Rica
# Authors: Daniel Garcia Vaglio degv364@gmail.com, Jeancarlo Hidalgo jeancahu@gmail.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

if [ "$(echo $* | grep -c '\-h')" -gt "0" ]
then
	echo "Help: blablablablablablabla"
	exit 0
fi

#OS=''
MODS="avispy roboview cmoc pyrovito vfl vfclik kbd-cart-cmd arcospyu"

#if [ "$(uname -a | grep -c Debian)" == 1 ]
#then
#	OS=debian
#elif [ "$(uname -a | grep -c Arch)" == 1 ]
#then
#	OS=arch
#else
#	echo "ERROR"
#fi

#echo $OS

mkdir -p $HOME/local/src
mkdir -p $HOME/local/DIR

for i in $MODS
do
        #echo $i
        echo "install $i"
        if [ -e $HOME/local/DIR/$i ]
        then
                echo 'It already exist!'
		#exit 0
        else
        cd $HOME/local/src
        git clone 'https://github.com/arcoslab/'"$i"'.git'
        cd $i
        make xstow_install
        fi


done

#robot_descriptions
echo "robot_descriptions"
if [ -e $HOME/local/src/robot_descriptions ]
then
        echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/robot_descriptions.git
fi

exit 0
