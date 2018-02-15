# -*- coding: utf-8 -*-

# Copyright (c) 2016-2018 Autonomous Robots and Cognitive Systems Laboratory
# Universidad de Costa Rica
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

# Ball_bot Class.

# Not implemented yet
# This is a simple class that stores the parameters for a kinematic model
# of a differential robot. Also computes the velocities that are necesary
# to work with the simulator core

from builtins import object


class Ball_bot(object):

    """ Ball bot model class

    Not implemented yet. This is a simle class that stores the parameters for
    a kinematic model of a ball bot. Also computes the velocities that are
    required to work with the simulator core
    """

    def __init__(self):
        raise NotImplementedError()

    def set_params(self, params):
        raise NotImplementedError()

    def central_velocity(self, req_velocity, angle):
        raise NotImplementedError()
