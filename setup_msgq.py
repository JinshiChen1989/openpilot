#!/usr/bin/env python3

from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np

ext = Extension(
    "msgq.ipc_pyx",
    sources=["msgq/ipc_pyx.pyx"],
    include_dirs=[".", "msgq", np.get_include()],
    libraries=["zmq"],
    language="c++",
    extra_compile_args=["-std=c++14"],
)

setup(
    ext_modules=cythonize([ext], build_dir="build"),
    zip_safe=False,
)