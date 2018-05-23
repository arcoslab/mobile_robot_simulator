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

from numpy import array, cos, sin
from vectorfield import vector_field
from kdl_helpers import add_delta, get_euler_zyx


class DifferentialRobot():
    """ Differential Robot class

    This is a simple class that stores the parameters for a kinematic model
    of a differential robot. Also computes the velocities that are necesary
    to work with the simulator core
    """

    def __init__(self, index, initial_pose, initial_vel, robot_dim,
                 robot_center, wheel_radius, wheel_distance, center_distance,
                 weight, max_vel):

        self.pose = initial_pose
        self.vel = initial_vel
        self.point_obstacle_constants = None
        self.line_obstacle_constants = None

        self.set_params(index, robot_dim, robot_center, wheel_radius,
                        wheel_distance, center_distance, weight, max_vel)

    def set_params(self, index, robot_dim, robot_center, wheel_radius,
                   wheel_distance, center_distance, weight, max_vel):
        """ Update the aprameters of a robot"""

        self.index = index  # tells the number of the robot
        self.robot_dim = robot_dim
        self.robot_center = robot_center

        self.r = wheel_radius
        self.b = wheel_distance
        self.d = center_distance

        # Inertial constants
        self.weight_force = weight
        self.object_mass = self.weight_force / 9.8
        self.object_rotational_inertia = self.object_mass * (
            self.robot_dim[0]**2 + self.robot_dim[1]**2) / 12.
        self.max_wheel_vel = max_vel

    def _wheel_velocity(self, descentralized_velocity, angle):
        """ Compute wheel velocities

        Using the kinematic model of a differential robot, calculate the
        required rotational velocities of the wheels to get a certain velocity
        at the descentralized point
        """

        right_wheel = (((self.d * cos(angle) - 0.5 * self.b * sin(angle)) *
                        descentralized_velocity[0] +
                        (self.d * sin(angle) + 0.5 * self.b * cos(angle)) *
                        descentralized_velocity[1]) / (self.d * self.r))
        left_wheel = (((self.d * cos(angle) + 0.5 * self.b * sin(angle)) *
                       descentralized_velocity[0] +
                       (self.d * sin(angle) - 0.5 * self.b * cos(angle)) *
                       descentralized_velocity[1]) / (self.d * self.r))

        # Limit max velocities on each wheel
        if left_wheel > self.max_wheel_vel:
            left_wheel = self.max_wheel_vel
        if right_wheel > self.max_wheel_vel:
            right_wheel = self.max_wheel_vel
        return [left_wheel, right_wheel]

    def _central_velocity(self, descentralized_point_vel, angle):
        """ Compute the robot's velocity at the geometric center

        Compute the velocity at the center of the robot, by knowing the
        required velocity and current angle
        """

        [phi_l, phi_r] = self._wheel_velocity(descentralized_point_vel, angle)
        central_velocity = array([0., 0., 0., 0., 0., 0.])
        central_velocity[0] = (
            (self.r * cos(angle)) / self.b) * (phi_r + phi_l)
        central_velocity[1] = (
            (self.r * sin(angle)) / self.b) * (phi_r + phi_l)
        central_velocity[5] = (self.r / self.b) * (phi_r - phi_l)
        return central_velocity

    def update_goal(self, goal_pose, goal_constant=1, damp_constant=0.5,
                    allowed_error=0.01):
        self.goal_pose = goal_pose
        self.goal_constant = goal_constant
        self.damp_constant = damp_constant
        self.allowed_error = allowed_error

    def update_obstacles(self,
                         point_obstacle_list=None,
                         line_obstacle_list=None,
                         point_obstacle_constants=None,
                         line_obstacle_constants=None):

        # Robots are considered point obstacles, so one should store all
        # obstacles except for self.
        point_num = len(point_obstacle_list)
        self.point_obstacle_list = [
            point_obstacle_list[i] for i in range(point_num)
            if i != self.index
        ]
        if self.point_obstacle_list is []:
            self.point_obstacle_list = None

        self.line_obstacle_list = line_obstacle_list

        # Robots are considered point obstacles, so one should store all
        # obstacles constants except for self.
        if point_obstacle_constants is not None:
            self.point_obstacle_constants = [
                point_obstacle_constants[i] for i in range(point_num)
                if i != self.index
            ]
            if self.point_obstacle_constants is []:
                self.point_obstacle_constants = None

        if line_obstacle_constants is not None:
            self.line_obstacle_constants = line_obstacle_constants

    def update_state(self, dt):
        """
        Update robots velocity and pose from the required velocity

        :param dt: time differential
        :returns: robots new pose
        """

        # Use vectorfield to compute required velocity
        req_vel = vector_field(
            dt, self.pose, self.vel, self.goal_pose, self.point_obstacle_list,
            self.point_obstacle_constants, self.line_obstacle_list,
            self.line_obstacle_constants, self.goal_constant,
            self.damp_constant, self.allowed_error)

        # Assuming that the robot only spins in Z
        z_angle, y_angle, x_angle = get_euler_zyx(self.pose[:3, :3])

        self.vel = self._central_velocity(req_vel, x_angle)

        add_delta(self.pose, self.vel, dt)

        return self.pose
