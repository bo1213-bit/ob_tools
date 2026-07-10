# Timestamp Sync Check

验证硬件同步的多台 Orbbec 相机在软件端到端的时间戳精度 < 1ms。

## 原理

- **硬件时间戳** (`OB_FRAME_METADATA_TYPE_TIMESTAMP`): 用于帧配对。硬件同步保证同一帧在所有相机上的硬件时间戳一致。
- **软件时间戳** (`std::chrono::system_clock::now()`): 在 SDK 回调入口立刻记录，用于对比验证。两台相机同一帧的软件时间戳差值 = 软件端到端精度。
- **Pairwise 对比**: 全矩阵配对，任意两台相机之间都做对比，输出最差情况。

## 编译

```bash
cd ob_tools
mkdir -p build && cd build
cmake .. -DOrbbecSDK_DIR=<path_to_orbbec_sdk>/lib
make -j$(nproc)
```

## 硬件连线

1. 用硬件同步线连接所有相机（菊花链或星型）
2. 所有相机通过 USB 连接到同一台主机
3. 增大 USB 缓冲区：

```bash
echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

## 使用

```bash
# 默认参数：30秒采集，30fps，640x400
./timestamp_sync_check

# 自定义参数
./timestamp_sync_check \
    --duration=60 \
    --sync-threshold=500 \
    --csv=result.csv \
    --histogram-all

# 查看帮助
./timestamp_sync_check --help
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--duration=N` | 30 | 采集时长（秒） |
| `--hw-threshold=N` | 500 | 硬件时间戳配对阈值（us） |
| `--sync-threshold=N` | 1000 | 软件同步精度判定阈值（us），即 <1ms |
| `--fps=N` | 30 | 采集帧率 |
| `--width=N` | 640 | 深度图宽度 |
| `--height=N` | 400 | 深度图高度 |
| `--csv=PATH` | (不导出) | CSV 文件路径 |
| `--histogram-bins=N` | 10 | 直方图 bin 数 |
| `--histogram-all` | false | 对所有 pair 画直方图 |

## 输出解读

### 每对相机统计

```
Device 0 (SN=xxx) vs Device 1 (SN=yyy)
  Pair count:   898
  Min diff:     -120.5 us
  Max diff:     350.2 us
  Abs max:      350.2 us
  Mean diff:    15.3 us
  Stddev:       45.7 us
  Verdict:      PASS (threshold=1.0 ms)
```

### 全局汇总

```
Global Summary
  Total pairs:       3
  Worst pair:        Device 0 vs Device 1
  Worst abs max:     350.2 us
  Sync threshold:    1.0 ms
  Global verdict:    PASS (< 1ms)
```

### 判定标准

- **PASS**: 所有 pair 的 |Max diff| < `--sync-threshold` (默认 1000us = 1ms)
- **FAIL**: 任意 pair 的 |Max diff| >= `--sync-threshold`

### CSV 格式

```csv
device_i,device_j,diff_us
0,1,15
0,1,-8
0,2,23
1,2,-5
...
```