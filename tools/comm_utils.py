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

# First step of the migration: create a wrapper that hides communication calls. To make it easier
# to remove YARP.

import mutiprocessing as m

# There are 3 YARP helper functions to wrap: new_port, readListPort, write_narray_port

# This is a global dictionary that stores the ports by name.
port_dict = {}

# This is a global variable that holds the name of unpaired conenctions 

# Use the same API as yarp_helper to hide the implementation change
def new_port(port_name, direction = "both", other_name, carrier='tcp', timeout=20.0):
    '''
    It creates a pair of ports, using a multiprocessing.Pipe. It only returns
    the one being requested. And stores the other one. When this function
    is called should look if the port alrady exists. 
    '''

    if port_name in port_dict:
        return port_dict[port_name]

    # The port has not been created, then create both
    
    my_port, other_port = Pipe()
    port_dict[port_name] = my_port
    port_dict[other_port] = other_port

    if direction!="both": print "Direction is ignored, update code"
    if carrier!="tcp": print "Carrier is ignored, uptdate code"
    if timeout!=20.0: print "Timeout is ignored, update code"

    return my_port

# Use the same API as yarp_helper to hide the implementation change
def readListPort(port, blocking=False):
    #FIXME: Not sure if I need to do some transformations first
    return port.recv()

# Use the same API as yarp_helper to hide the implementation change
def write_narray_port(port, narray):
    #FIXME: Not sure if I need to do some transformations fist
    port.send(narray)
