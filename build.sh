#!/bin/bash

make O=out ARCH=arm64 r8q_defconfig

make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CROSS_COMPILE=aarch64-linux-gnu- \
                      
make -j$(nproc --all) O=out \
                      ARCH=arm64 \
                      CROSS_COMPILE=aarch64-linux-gnu- \
                      
if [[ -f "$Image" ]]; then
    rm AnyKernel3/*.zip > /dev/null 2>&1
    cp $Image AnyKernel3/Image.gz
    cd AnyKernel3
    zip -r9 meow-r8q.zip .
fi

