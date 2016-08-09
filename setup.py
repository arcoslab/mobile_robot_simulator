#!/usr/bin/env python
# Copyright (c) 2011 Autonomous Robots and Cognitive Systems Laboratory, Universidad de Costa Rica
# Authors: Daniel Garcia Vaglio degv364@gmail.com
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
from distutils.core import setup


setup(name='mobile_robot_simulator',
      version='0.1',
      description='Mobile Robot Simulator',
      author='Daniel Garcia Vaglio',
      author_email='degv364@gmail.com',
      url='http://www.arcoslab.org/',
      package_dir={'mobile_robot_simulator': ''},
      packages=['mobile_robot_simulator', 'mobile_robot_simulator.robot_models', 'mobile_robot_simulator.tools', 'mobile_robot_simulator.vectorfield'],
      scripts=['core.py','tools/goal_obstacle.py' ]
     )
