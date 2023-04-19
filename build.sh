#!/bin/bash

make O=out ARCH=arm64 r8q_eur_openx_defconfig

PATH=$PATH:/home/mao-server/Downloads/clang-r475365b/bin/

make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC=/home/mao-server/downloads/clang-r475365b/bin/clang \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE=/home/mao-server/Downloads/gcc-linaro-7.4.1/gcc/bin/aarch64-linux-gnu- \
                      r8q_eur_openx_defconfig

make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CLANG_TRIPLE=aarch64-linux-gnu- \
                      CROSS_COMPILE=/home/mao-server/Downloads/gcc-linaro-7.4.1/gcc/bin/aarch64-linux-gnu- \
                      Image.gz
