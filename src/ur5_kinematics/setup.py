## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from os.path import basename, dirname, abspath
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[basename(dirname(abspath(__file__)))],
    package_dir={'': 'src'})

setup(**setup_args)