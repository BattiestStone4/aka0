#!/bin/bash

# 设置交叉编译工具链
export CC=riscv64-unknown-linux-musl-gcc
export CXX=riscv64-unknown-linux-musl-g++

# 设置SDK路径，请根据实际情况修改
TPU_SDK_PATH=${TPU_SDK_PATH:-"/opt/cvitek_tpu_sdk"}
OPENCV_PATH=${OPENCV_PATH:-"/opt/cvitek_tpu_sdk/opencv"}

echo "Using TPU_SDK_PATH: $TPU_SDK_PATH"
echo "Using OPENCV_PATH: $OPENCV_PATH"

# 清理之前的构建
rm -rf build_riscv_debug
mkdir -p build_riscv_debug
cd build_riscv_debug

# 配置cmake
cmake .. \
    -DCMAKE_BUILD_TYPE=Debug \
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
    -DLOG_LEVEL=debug

# 编译
make -j$(nproc)

echo ""
echo "=== Build Results ==="
for bin in tennis capture camera_test arm_test motor_test; do
    if [ -f "$bin" ]; then
        size=$(ls -lh "$bin" | awk '{print $5}')
        echo "  $bin  $size"
    else
        echo "  $bin  MISSING"
    fi
done

echo ""
echo "Done! Binaries in: $(pwd)"
