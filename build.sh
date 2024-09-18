#!/bin/bash
if [ -d "build" ]; then
  rm -rf build
fi

if [ -d "bin" ]; then
  rm -rf bin
fi

mkdir -p build
mkdir -p bin
cd build
cmake ..
make
cp devel/lib/rog_map/rog_map ../bin/
cd ..

if [ -d "build" ]; then
  rm -rf build
fi

chmod +x bin/rog_map
