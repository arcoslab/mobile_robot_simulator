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


#install avispy
echo "install avispy"
cd $HOME/local/src
git clone https://github.com/arcoslab/avispy.git
cd avispy
make xstow_install

#install roboview
echo "install roboview"
cd $HOME/local/src
git clone https://github.com/arcoslab/roboview.git
cd roboview
make xstow_install

#install cmoc
echo "install cmoc"
cd $HOME/local/src
git clone https://github.com/arcoslab/cmoc.git
cd cmoc
make xstow_install

#install pyrovito
echo "install pyrovito"
cd $HOME/local/src
git clone https://github.com/arcoslab/pyrovito.git
cd pyrovito
make xstow_install

#install vfl
echo "vfl"
cd $HOME/local/src
git clone https://github.com/arcoslab/vfl.git
cd vfl
make xstow_install

#install vfclick
echo "install vfclick"
cd $HOME/local/src
git clone https://github.com/arcoslab/vfclik.git
cd vfclik
make xstow_install

#install kdb cart cmd
echo "install kdb cart cmd"
cd $HOME/local/src
git clone https://github.com/arcoslab/kbd-cart-cmd.git
cd kbd-cart-cmd
make xstow_install

#install arcospyu
echo "install arcospyu"
cd $HOME/local/src
git clone https://github.com/arcoslab/arcospyu.git
cd arcospyu
make xstow_install

#robot_descriptions
echo "robot_descriptions"
cd $HOME/local/src
git clone https://github.com/arcoslab/robot_descriptions.git


exit 0
