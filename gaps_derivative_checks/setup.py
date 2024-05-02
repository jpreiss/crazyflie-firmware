# Available at setup time due to pyproject.toml
import eigenpip
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

__version__ = "0.2.1"

eigen = eigenpip.get_include()
include_dirs = [
    eigen,
    eigen + "/unsupported",
    "../src/modules/interface",
    "../src/modules/src/controller",
]

deps = [
    "../src/modules/interface/gapsquad.h",
    "../src/modules/src/controller/gapsquad.hpp",
]

ext_modules = [
    Pybind11Extension("gapsquad",
        ["bindings.cpp"],
        depends=deps,
        include_dirs=include_dirs,
        define_macros=[("EIGEN_NO_MALLOC", None)],
        cxx_std=14,
        ),
]

setup(
    name="gapsquad",
    version=__version__,
    author="James A. Preiss",
    author_email="jamesalanpreiss@gmail.com",
    ext_modules=ext_modules,
    packages=["gapsquad"],
    package_data={
        "gapsquad": ["gapsquad/bindings.cpp"] + deps
    },
    install_requires=["eigenpip"],
    zip_safe=False,  # TODO: Understand.
    python_requires=">=3.7",
)
