# -*- coding: utf-8 -*-
# Copyright (c) 2012 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Federico Ruiz-Ugalde <memeruiz@gmail.com>
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

# Braught from pyrovito and adapted to Python3

import yarp
from time import sleep
cstyle = yarp.ContactStyle()
cstyle.persistent = True


class RoboviewerObjects():
    def __init__(self, portbasename, roboviewerportbasename, counter=-1,
                 wait_connection=False):
        """
        Initialize the communication interface with pyrovito

        :param portbasename: base name of the local module
        :param roboviewerportbasename: pyrovito basee name
        :param counter: initial counter value
        :param wait_connection: If roboviewer should wait for connections
        """
        self.out_port = yarp.BufferedPortBottle()
        self.out_port.open(portbasename + "/roboviewer_out")
        yarp.Network.connect(portbasename + "/roboviewer_out",
                             roboviewerportbasename + "/objects:i", cstyle)
        if wait_connection:
            while not yarp.Network.isConnected(
                    portbasename +
                    "/roboviewer_out", roboviewerportbasename + "/objects:i"):
                sleep(0.01)
        self.counter = counter

    def create_object(self, object_type):
        """
        Tell pyrovito to create an object

        :param object_type: name of the type in avispy's object_lib
        :return: The assigned object ID
        """
        self.counter += 1
        bottle = self.out_port.prepare()
        bottle.clear()
        bottle_list = bottle.addList()
        bottle_list.addInt(self.counter)
        bottle_list.addString("type")
        bottle_list.addString(object_type)
        self.out_port.write(True)
        return (self.counter)

    def send_prop(self, object_id, prop_name, prop_data):
        """
        Tell pyrovito to update an object's state

        :param object_id: number that identifies the object
        :param prop_name: name of the parameter to change
        :param prop_data: the parameter itself
        """
        bottle = self.out_port.prepare()
        bottle.clear()
        bottle_list = bottle.addList()
        bottle_list.addInt(object_id)
        bottle_list.addString(prop_name)
        for i in prop_data:
            bottle_list.addDouble(i)
        self.out_port.write(True)

    def send_prop_multi(self, object_ids, prop_names, prop_datas):
        """
        Same as send_prop but for multiple objects
        """
        bottle = self.out_port.prepare()
        bottle.clear()
        for object_id, prop_name, prop_data in zip(object_ids, prop_names,
                                                   prop_datas):
            bottle_list = bottle.addList()
            bottle_list.clear()
            bottle_list.addInt(object_id)
            bottle_list.addString(prop_name)
            for i in prop_data:
                bottle_list.addDouble(i)
        self.out_port.write(True)
