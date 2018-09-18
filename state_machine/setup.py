from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[	'scripts/smach_sm.py', 
  			'scripts/smach_sm.py']
)

setup(**d)
