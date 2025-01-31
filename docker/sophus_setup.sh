#!/bin/bash

# Install vcpkg, then use vcpkg to install sophus
. <(curl https://aka.ms/vcpkg-init.sh -L) && sleep 3 && vcpkg-shell new --application && \
    vcpkg-shell add port sophus && vcpkg-shell install