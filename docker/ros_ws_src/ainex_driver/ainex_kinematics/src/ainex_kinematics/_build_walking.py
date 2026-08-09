#!/usr/bin/env python3
"""One-shot Cython build script for walking_module.pyx — run inside ainex container."""
import sys, os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np

ext = Extension(
    "walking_module",
    sources=["walking_module.pyx"],
    include_dirs=[np.get_include()],
    extra_compile_args=["-O2", "-ffast-math"],
)

sys.argv = ["setup.py", "build_ext", "--inplace"]
setup(ext_modules=cythonize([ext], language_level=3))
