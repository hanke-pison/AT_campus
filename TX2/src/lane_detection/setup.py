from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['lane_detection'],
    package_dir={'':'scripts'},
)

setup(**setup_args)
