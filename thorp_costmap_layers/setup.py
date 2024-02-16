#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['thorp_costmap_layers']
d['package_dir'] = {'': 'src'}
d['install_requires'] = []

setup(**d)
