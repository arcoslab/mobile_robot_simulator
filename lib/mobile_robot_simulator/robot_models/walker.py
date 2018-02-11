#!/usr/bin/env python
# Copyright (c) 2011 Autonomous Robots and Cognitive Systems Laboratory
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

# Walker robot Class.

# Not implemented yet
# This is a simple class that stores the parameters for a kinematic model
# of a differential robot. Also computes the velocities that are necesary
# to work with the simulator core


from __future__ import print_function
from builtins import object


class Differential_robot(object):
    def __init__(self):
        print("not implemented")

    def set_params(self, params):
        print("not implemented")

    def central_velocity(self, req_velocity, angle):
        return req_velocity
