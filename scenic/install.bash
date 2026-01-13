#!/bin/bash

cd ~/clipper
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake --build .
sudo cmake --install . --prefix /usr/local/

cd ~/scenic
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake --build .
sudo cmake --install . --prefix /usr/local/ 
