#!/bin/bash

TC_PATH="/home/mao-server/Downloads/"

BUILD_ENV="CC=$(echo $TC_PATH)clang-r475365b/bin/clang CLANG_TRIPLE=aarch64-linux-gnu- CROSS_COMPILE=$(echo $TC_PATH)gcc-linaro-7.4.1/gcc/bin/aarch64-linux-gnu-"

make O=out ARCH=arm64 $BUILD_ENV r8q_defconfig

make -j$(nproc --all) O=out ARCH=arm64 $BUILD_ENV Image.gz

make -j$(nproc --all) O=out ARCH=arm64 $BUILD_ENV dtbs

DTB_OUT="out/arch/arm64/boot/dts/vendor/qcom"

cat $DTB_OUT/*.dtb > AnyKernel3/dtb
