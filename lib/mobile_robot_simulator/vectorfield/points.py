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

# This script takes the position of the goal, and the position of the
# obstacles and computes the direction a robot in a particular point needs.

from builtins import range
from numpy import array
from numpy.linalg import norm


def vectorfield(goal_pos,
                obstacle_pos_list,
                robot_pos,
                robot_vel,
                dt,
                goal_constant=1.,
                obstacle_constant=1.,
                damp_constant=0.5):
    obstacle_cant = len(obstacle_pos_list)

    # get distances
    goal_distance = norm(goal_pos - robot_pos)
    obstacle_distance_list = [0] * obstacle_cant

    # calculate forces
    goal_force = goal_constant * (goal_pos - robot_pos) / goal_distance
    obstacle_force = 0

    for i in range(obstacle_cant):
        obstacle_distance_list[i] = norm(obstacle_pos_list[i] - robot_pos)
        obstacle_force += obstacle_constant * (
            robot_pos - obstacle_pos_list[i]) / (obstacle_distance_list[i]**3)

    if goal_distance < 1:
        damp_constant = goal_distance * (damp_constant - 1) / (1 - 0.3) + (
            1 - 0.3 * damp_constant)

    damp_force = -damp_constant * robot_vel

    robot_vel += (goal_force + obstacle_force) * dt + damp_force
    if goal_distance < 0.3:
        robot_vel = array([0., 0., 0.])
    return robot_vel
