#!/bin/bash
set -e

# 清理旧输出
rm -rf out build.log
mkdir -p out

echo "--- 正在配置内核 ---"
make ARCH=arm64 O=out \
    HOSTCC=gcc HOSTLD=ld \
    LLVM=1 LLVM_IAS=1 \
    CC=clang \
    LD=ld.lld \
    CLANG_TRIPLE=aarch64-linux-android \
    CROSS_COMPILE=aarch64-linux-android- \
    gki_defconfig

echo "--- 正在开始编译 ---"
make ARCH=arm64 O=out \
    HOSTCC=gcc HOSTLD=ld \
    LLVM=1 LLVM_IAS=1 \
    CC=clang \
    LD=ld.lld \
    CLANG_TRIPLE=aarch64-linux-android \
    CROSS_COMPILE=aarch64-linux-android- \
    -j$(nproc --all) 2>&1 | tee build.log
