from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'xyz_bt_lib',
        'xyz_bt_lib.core',
        'xyz_bt_lib.blackboard',
        'xyz_bt_lib.adapters',
        'xyz_bt_lib.behaviours',
        'xyz_bt_lib.behaviours.L1_perception',
        'xyz_bt_lib.behaviours.L2_locomotion',
        'xyz_bt_lib.behaviours.L3_system',
    ],
    package_dir={'': 'src'},
)
setup(**d)
