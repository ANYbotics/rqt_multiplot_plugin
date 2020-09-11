#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
packages=['rqt_multiplot'],
package_dir={'': 'src'},
scripts=['scripts/rqt_multiplot'],
)
setup(**d)
