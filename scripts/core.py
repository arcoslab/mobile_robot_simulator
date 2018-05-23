#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2016-2018 Autonomous Robots and Cognitive Systems Laboratory
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

# This program was developed by Daniel Garcia Vaglio (degv364@gmail.com).
# Inspired from a program by Federico Ruiz.


# from pyrovito.pyrovito_utils import Roboviewer_objects

import argparse
import json
import signal
from mobile_robot_simulator.manager import Manager

""" This is a program that will simulate a differential robot."""


def main():
    parser = argparse.ArgumentParser
    parser.add_argument(
        "-c",
        "--conf_file",
        dest="conf_file_name",
        default="conf_file.json",
        help="Configuration file"
    )

    args = parser.parse_args()

    with open(args.conf_file_name, "r") as conf_file:
        conf = json.load(conf_file)

    print("Starting simulation")

    manager = Manager(conf)

    signal.signal(signal.SIGINT, manager.terminate())
    signal.signal(signal.SIGTERM, manager.terminate())

    manager.loop()

    print("Simulation ended")


if __name__ == "__main__":
    main()
