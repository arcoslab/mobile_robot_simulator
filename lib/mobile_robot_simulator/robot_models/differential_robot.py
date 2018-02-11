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

# Differential Robot Class.
# This is a simple class that stores the parameters for a kinematic model
# of a differential robot. Also computes the velocities that are necesary
# to work with the simulator core

from __future__ import division
from builtins import object
from past.utils import old_div
from numpy import array, cos, sin


class Differential_robot(object):
    def __init__(self):
        # dimensions
        self.robot_dim = [.5, 0.5, .3]
        self.robot_center = array([0., 0., 0])
        self.robot_planes = [
            # normal point
            [(1., 0., 0), (old_div(self.robot_dim[0], 2.), 0., 0.)],
            [(-1., 0., 0), (old_div(-self.robot_dim[0], 2.), 0., 0.)],
            [(0., 1., 0), (0., old_div(self.robot_dim[1], 2.), 0.)],
            [(0., -1., 0), (0., old_div(-self.robot_dim[1], 2.), 0.)],
            [(0., 0., 1), (0., 0., old_div(self.robot_dim[2], 2.))],
            [(0., 0., -1), (0., 0., old_div(-self.robot_dim[2], 2.))]
        ]

        self.surface_object_face = 5
        self.r = 1  # Wheel radius
        self.b = self.robot_dim[0]  # Distance between wheels
        self.d = old_div(self.robot_dim[
            1], 2.0)  # Distance between robot center and descentralized point

        # Inertial constants
        self.weight_force = 2.
        self.object_mass = old_div(self.weight_force, 9.8)
        self.object_rotational_inertia = self.object_mass * (
            self.robot_dim[0]**2 + self.robot_dim[1]**2) / 12.
        self.max_wheel_vel = 100

    def set_params(self, robot_dim, robot_center, wheel_radius, wheel_distance,
                   center_distance, weight, max_vel):
        # set the parameters on a robot that was already initialized
        self.robot_dim = robot_dim
        self.robot_center = robot_center
        self.robot_planes = [
            # normal point
            [(1., 0., 0), (old_div(self.robot_dim[0], 2.), 0., 0.)],
            [(-1., 0., 0), (old_div(-self.robot_dim[0], 2.), 0., 0.)],
            [(0., 1., 0), (0., old_div(self.robot_dim[1], 2.), 0.)],
            [(0., -1., 0), (0., old_div(-self.robot_dim[1], 2.), 0.)],
            [(0., 0., 1), (0., 0., old_div(self.robot_dim[2], 2.))],
            [(0., 0., -1), (0., 0., old_div(-self.robot_dim[2], 2.))]
        ]

        self.surface_object_face = 5
        self.r = wheel_radius
        self.b = wheel_distance
        self.d = center_distance

        # Inertial constants
        self.weight_force = weight
        self.object_mass = old_div(self.weight_force, 9.8)
        self.object_rotational_inertia = self.object_mass * (
            self.robot_dim[0]**2 + self.robot_dim[1]**2) / 12.
        self.max_wheel_vel = max_vel

    def wheel_velocity(self, descentralized_velocity, angle):
        # Compute wheel velocities from the required velocity of the
        # descentralized point, and the current angle of the robot.
        right_wheel = old_div(((self.d * cos(angle) - 0.5 * self.b *
                                sin(angle)) * descentralized_velocity[0] +
                               (self.d * sin(angle) + 0.5 * self.b *
                                cos(angle)) * descentralized_velocity[1]),
                              (self.d * self.r))
        left_wheel = old_div(((self.d * cos(angle) + 0.5 * self.b
                               * sin(angle)) * descentralized_velocity[0] +
                              (self.d * sin(angle) - 0.5 * self.b
                               * cos(angle)) * descentralized_velocity[1]),
                             (self.d * self.r))

        # Limit max velocities on each wheel
        if left_wheel > self.max_wheel_vel:
            left_wheel = self.max_wheel_vel
        if right_wheel > self.max_wheel_vel:
            right_wheel = self.max_wheel_vel
        return [left_wheel, right_wheel]

    def central_velocity(self, descentralized_point_vel, angle):
        # Compute the velocity at the center of the robot, by knowing the
        # required velocity and current angle
        [phi_l, phi_r] = self.wheel_velocity(descentralized_point_vel, angle)
        central_velocity = array([0., 0., 0., 0., 0., 0.])
        central_velocity[0] = (old_div((self.r * cos(angle)), self.b)) * (
            phi_r + phi_l)
        central_velocity[1] = (old_div((self.r * sin(angle)), self.b)) * (
            phi_r + phi_l)
        central_velocity[5] = (old_div(self.r, self.b)) * (phi_r - phi_l)
        return central_velocity
