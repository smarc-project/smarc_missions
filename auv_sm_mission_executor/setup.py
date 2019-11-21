from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[ 'scripts/auv_mission_executor.py' ],
  packages=['auv_sm_mission_executor'],
  package_dir={'': 'src'})

setup(**d)
