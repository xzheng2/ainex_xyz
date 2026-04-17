from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'ainex_bt_edu',
        'ainex_bt_edu.behaviours',
        'ainex_bt_edu.behaviours.L1_perception',
        'ainex_bt_edu.behaviours.L2_locomotion',
        'ainex_bt_edu.behaviours.L3_mission',
        'ainex_bt_edu.input_adapters',
    ],
    package_dir={'': 'src'},
)
setup(**d)
