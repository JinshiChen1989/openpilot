#!/usr/bin/env python3

from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np

ext = Extension(
    "msgq.visionipc.visionipc_pyx",
    sources=["msgq/visionipc/visionipc_pyx.pyx"],
    include_dirs=[".", "msgq", "msgq/visionipc", np.get_include()],
    language="c++",
    extra_compile_args=["-std=c++14"],
)

setup(
    ext_modules=cythonize([ext], build_dir="build"),
    zip_safe=False,
)