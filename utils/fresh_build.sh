#!/bin/bash
cd ~/anahita_ws
rm -r build devel
./src/Anahita/utils/travis_build.sh
catkin_make --pkg hardware_arduino
