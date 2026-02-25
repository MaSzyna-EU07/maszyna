#!/usr/bin/env bash

echo "Setting up vcpkg packages"
cd ref/vcpkg
./bootstrap-vcpkg.sh
./vcpkg install directx-dxc:x64-linux
cd ../..
