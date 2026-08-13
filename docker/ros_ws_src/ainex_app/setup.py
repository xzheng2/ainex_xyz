#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# No Python package under src/ any more — button_node.py is a standalone script
# installed by CMakeLists.txt, so there is nothing to declare here.
setup(**generate_distutils_setup())
