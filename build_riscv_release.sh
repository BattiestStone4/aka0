#!/bin/bash

# 设置交叉编译工具链
export CC=riscv64-unknown-linux-musl-gcc
export CXX=riscv64-unknown-linux-musl-g++

# 设置SDK路径，请根据实际情况修改
TPU_SDK_PATH=${TPU_SDK_PATH:-"/home/ajax/Proj/OS/sg2002/cvitek_tpu_sdk"}
OPENCV_PATH=${OPENCV_PATH:-"/home/ajax/Proj/OS/sg2002/cvitek_tpu_sdk/opencv"}

echo "Using TPU_SDK_PATH: $TPU_SDK_PATH"
echo "Using OPENCV_PATH: $OPENCV_PATH"

# 清理之前的构建
rm -rf build_riscv_release
mkdir -p build_riscv_release
cd build_riscv_release

# 配置cmake - Release版本
# LOG_LEVEL可以通过环境变量设置，默认为warn
# 使用方法: LOG_LEVEL=debug ./build_riscv_release.sh
LOG_LEVEL=${LOG_LEVEL:-warn}

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=riscv64 \
    -DCMAKE_C_COMPILER=${CC} \
    -DCMAKE_CXX_COMPILER=${CXX} \
    -DCMAKE_C_FLAGS="-O2 -mcpu=c906fdv -mabi=lp64d" \
    -DCMAKE_CXX_FLAGS="-O2 -mcpu=c906fdv -mabi=lp64d" \
    -DCMAKE_EXE_LINKER_FLAGS="-Wl,--dynamic-linker=/lib/ld-musl-riscv64v0p7_xthead.so.1" \
    -DCMAKE_CROSSCOMPILING=ON \
    -DTPU_SDK_PATH=${TPU_SDK_PATH} \
    -DOPENCV_PATH=${OPENCV_PATH} \
    -DLOG_LEVEL=${LOG_LEVEL}

# 编译
make -j$(nproc)

echo "Release version compiled successfully!"
echo "Executable: $(pwd)/tennis"
echo "Log level: ${LOG_LEVEL} (default: warn, override with LOG_LEVEL=xxx)"
