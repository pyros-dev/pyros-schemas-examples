#!/usr/bin/env python
import setuptools

# Ref : https://packaging.python.org/single_source_version/#single-sourcing-the-version
with open('pyros_httpbin/_version.py') as vf:
    exec(vf.read())


setuptools.setup(name='pyros_httpbin',
    version=__version__,
    description='Pyros Web client for httpbin.org',
    url='http://github.com/asmodehn/pyros-schemas-examples',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='MIT',
    packages=[
        'pyros_httpbin',
    ],
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    # Reference for optional dependencies : http://stackoverflow.com/questions/4796936/does-pip-handle-extras-requires-from-setuptools-distribute-based-sources
    install_requires=[

    ],
    zip_safe=False,  # TODO testing...
)




