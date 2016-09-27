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

OS=''

if [ "$(uname -a | grep -c Debian)" == 1 ]
then
	OS=debian
elif [ "$(uname -a | grep -c Arch)" == 1 ]
then
	OS=arch
else
	echo "ERROR"
fi

#echo $OS

mkdir -p $HOME/local/src
mkdir -p $HOME/local/DIR

#install avispy
echo "install avispy"
if [ -e $HOME/local/DIR/avispy ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/avispy.git
cd avispy
make xstow_install
fi

#install roboview
echo "install roboview"
if [ -e $HOME/local/DIR/roboview ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/roboview.git
cd roboview
make xstow_install
fi

#install cmoc
echo "install cmoc"
if [ -e $HOME/local/DIR/cmoc ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/cmoc.git
cd cmoc
make xstow_install
fi

#install pyrovito
echo "install pyrovito"
if [ -e $HOME/local/DIR/pyrovito ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/pyrovito.git
cd pyrovito
make xstow_install
fi

#install vfl
echo "install vfl"
if [ -e $HOME/local/DIR/vfl ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/vfl.git
cd vfl
make xstow_install
fi

#install vfclick
echo "install vfclick"
if [ -e $HOME/local/DIR/vfclik ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/vfclik.git
cd vfclik
make xstow_install
fi

#install kdb cart cmd
echo "install kdb cart cmd"
if [ -e $HOME/local/DIR/kbd-cart-cmd ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/kbd-cart-cmd.git
cd kbd-cart-cmd
make xstow_install
fi

#install arcospyu
echo "install arcospyu"
if [ -e $HOME/local/DIR/arcospyu ]
then
	echo 'It already exist!'
else
cd $HOME/local/src
git clone https://github.com/arcoslab/arcospyu.git
cd arcospyu
make xstow_install
fi

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
