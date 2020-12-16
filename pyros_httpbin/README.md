# pyros-rosclient
Package to use Pyros as a ROS client to other multiprocess architecture ( Web or others )

The location & name & structure of this repo might change soon. It s still early deevlopment and quite unstable.

In particular we need to find out how to mix and match:

- existing pyros
- ros extensions for pyros (as opposed to zmp extensions for example)
- library to easily build ros-web client (and zmp-web ?)
- test for a ros-web client using httpbin
- gopher software application (for yujinrobot)

---
## test

mkdir build

cd build & cmake ../ & make & source devel/setup.bash & make test

or

py.test ../tests/test_httpbin.py -vv


## test for memory leak

source devel/setup.bash & cd ../scripts & ./leaktest.sh

or

roslaunch pyros_httpbin leaktest.launch --screen

## Troubleshooting

easy_install netaddr, when occur no module netaddr

git clone https://github.com/pyros-dev/pyros-setup.git & cd pyros-setup & python setup.py install, when occur no module pyros_setup

cd pyros_schemas-examples/build & source devel/setup.bash, when occur path error

## Docker

docker run -it --rm --name yujin-d3-devel-memory_leak_test --network=host zaxrok/yujin-d3:devel-memory_leak_test bash

