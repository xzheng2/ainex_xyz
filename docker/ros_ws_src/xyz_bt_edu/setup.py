from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'xyz_bt_edu',
        'xyz_bt_edu.behaviours',
        'xyz_bt_edu.behaviours.L1_perception',
        'xyz_bt_edu.behaviours.L2_locomotion',
        'xyz_bt_edu.input_adapters',
    ],
    package_dir={'': 'src'},
)
setup(**d)
