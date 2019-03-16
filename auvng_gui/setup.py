from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['auvng_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/auvng_gui']
)

setup(**d)
