from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[	'scripts/base_executor.py',
  			'scripts/fifo_task_executor.py',
  			'scripts/sm_base_executor.py']
)

setup(**d)