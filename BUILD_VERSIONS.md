# 构建版本说明

## 版本选择

项目提供两个编译脚本，主要区别在于**默认日志级别**，但都支持运行时覆盖。

### 1. Debug脚本 (默认详细日志)
**编译脚本**: `./build_riscv_debug.sh`
**可执行文件**: `build_riscv_debug/tennis`
**默认日志级别**: DEBUG (显示所有信息)

**输出内容**:
- 帧详细信息 (Frame编号)
- VI采集详情 (GetFrame, VPSS_Resize, Cvt时间)
- 性能分析 (Read, Preprocess, Inference, Postprocess时间)
- 检测信息 (球的位置、置信度、面积)
- 状态机信息 (追球、对齐、抓取等状态)
- 电机控制信息 (转向、前进等)
- FPS信息

**适用场景**:
- 开发调试
- 性能分析
- 问题排查

### 2. Release脚本 (默认简洁日志)
**编译脚本**: `./build_riscv_release.sh`
**可执行文件**: `build_riscv_release/tennis`
**默认日志级别**: WARN (只显示警告和错误)

**输出内容**:
- FPS信息 (帧率和平均帧率)
- 警告信息 (如获取帧失败)
- 错误信息

**适用场景**:
- 正式运行
- 性能测试 (避免日志输出的性能影响)
- 长时间运行

## 编译方法

### 基础用法
```bash
cd /home/ajax/Proj/OS/sg2002/aka0

# Debug脚本（默认debug级别）
./build_riscv_debug.sh

# Release脚本（默认warn级别）
./build_riscv_release.sh
```

### 高级用法：自定义日志级别

**两个脚本都支持**通过环境变量覆盖默认的日志级别：

```bash
# 使用debug脚本，但编译成info级别
LOG_LEVEL=info ./build_riscv_debug.sh

# 使用debug脚本，但编译成warn级别（简洁版）
LOG_LEVEL=warn ./build_riscv_debug.sh

# 使用release脚本，但编译成debug级别（详细版）
LOG_LEVEL=debug ./build_riscv_release.sh

# 使用release脚本，编译成error级别（最简洁）
LOG_LEVEL=error ./build_riscv_release.sh
```

## 日志级别说明

| 级别 | 宏定义值 | 显示内容 | 使用场景 |
|------|--------|----------|---------|
| DEBUG | 0 | 所有信息 | 深度调试、性能分析 |
| INFO  | 1 | 一般信息、FPS | 日常开发、功能测试 |
| WARN  | 2 | 警告、FPS | 性能测试、正式运行 |
| ERROR | 3 | 仅错误 | 生产环境、问题排查 |

## 使用示例

### 场景1：开发调试
```bash
# 使用debug脚本，默认debug级别
./build_riscv_debug.sh
./build_riscv_debug/tennis model.cvimodel
```

### 场景2：性能测试（需要详细日志分析）
```bash
# 使用release脚本，但指定debug级别
LOG_LEVEL=debug ./build_riscv_release.sh
./build_riscv_release/tennis model.cvimodel
```

### 场景3：正式运行（简洁输出）
```bash
# 使用release脚本，默认warn级别
./build_riscv_release.sh
./build_riscv_release/tennis model.cvimodel
```

### 场景4：快速切换日志级别
```bash
# 编译多个版本，不同日志级别
LOG_LEVEL=debug ./build_riscv_debug.sh && mv build_riscv_debug build_debug
LOG_LEVEL=warn ./build_riscv_debug.sh && mv build_riscv_debug build_warn
LOG_LEVEL=error ./build_riscv_debug.sh && mv build_riscv_debug build_error

# 根据需要选择运行
./build_debug/tennis model.cvimodel    # 最详细
./build_warn/tennis model.cvimodel     # 简洁
./build_error/tennis model.cvimodel    # 最简洁
```

## 性能影响

- DEBUG级别：由于大量日志输出，可能对性能有轻微影响 (约1-2ms/帧)
- WARN级别：最小化日志输出，性能最优
- ERROR级别：几乎没有日志开销

## 推荐使用

- **开发阶段**：使用 `./build_riscv_debug.sh` (默认DEBUG级别)
- **性能测试**：使用 `LOG_LEVEL=debug ./build_riscv_release.sh` (详细日志分析)
- **正式运行**：使用 `./build_riscv_release.sh` (默认WARN级别)
- **生产环境**：使用 `LOG_LEVEL=error ./build_riscv_release.sh` (最稳定)
