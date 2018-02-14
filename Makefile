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


NAME=mobile_robot_simulator
#NAME=cmoc
#PREFIX ?= ${HOME}/local/lib/${NAME}
PREFIX ?= ${HOME}/local/
DEB_TARGET=python-mobile_robot_simulator_0.1-1_all.deb

all:
	echo "Does nothing, try make install"

install:
	python setup.py install --prefix=${PREFIX}

xstow_install: install
	cd ${PREFIX}/../ && xstow ${NAME}

xstow_uninstall:
	cd ${PREFIX}/../ && xstow -D ${NAME} && rm -rf ${NAME}

config:
	bash config.sh

purge:
	bash config.sh --purge


%.deb:
	python setup.py --command-packages=stdeb.command bdist_deb

deb: deb_dist/${DEB_TARGET}

deb_install: deb_dist/${DEB_TARGET}
	cd deb_dist && sudo dpkg -i *.deb

clean:
	python setup.py clean
	rm -rf build/ deb_dist/
