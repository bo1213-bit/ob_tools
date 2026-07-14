# Timestamp Sync Check

验证硬同步（Hardware Triggering）条件下多台 Orbbec 相机的时间戳同步精度。同时采集 Depth 和 Color 双流，输出三类对比：

- **同设备跨流**：Depth vs Color（设备时间戳差 + 软件时间戳差）
- **跨设备同流 Depth**：Device[i].Depth vs Device[j].Depth
- **跨设备同流 Color**：Device[i].Color vs Device[j].Color

## 架构

```
src/
├── main.cpp                入口：解析参数 → 采集 → 分析 → 输出
├── data_collector.cpp      模块1：枚举设备、配置硬同步、采集双流帧数据
├── sync_analyzer.cpp       模块2：帧配对、三类对比、统计、CSV 导出
└── visualize_sync.py       模块3：读取 CSV 生成箱型图 + HTML 汇总报告
```

## 编译

```bash
cd ob_tools
mkdir -p build && cd build
cmake .. -DOrbbecSDK_DIR=<path_to_orbbec_sdk>/lib
make -j$(nproc)
```

## 硬件连线

1. 用硬件同步线连接所有相机（星型拓扑：PRIMARY 的 SYNC_OUT 接所有 SECONDARY 的 SYNC_IN）
2. 所有相机通过 USB/GMSL 连接到同一台主机
3. 增大 USB 缓冲区：

```bash
echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

## 使用

### C++ 工具

```bash
# 基础用法：30 秒采集，848×480@30fps，Depth+Color 双流
./timestamp_sync_check --csv=result.csv

# 自定义帧率和时长
./timestamp_sync_check --fps=15 --duration=60 --csv=result.csv

# 只采集 Depth 流
./timestamp_sync_check --no-color --csv=result_depth.csv

# 查看帮助
./timestamp_sync_check --help
```

### Python 可视化

```bash
# 安装依赖
pip install numpy matplotlib

# 生成图表
python src/visualize_sync.py result.csv --output ./charts
# 浏览器打开 charts/summary.html
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--duration=N` | 30 | 采集时长（秒） |
| `--fps=N` | 30 | 帧率 |
| `--width=N` | 848 | 分辨率宽度 |
| `--height=N` | 480 | 分辨率高度 |
| `--hw-threshold=N` | 500 | 硬件时间戳配对阈值（us），两帧 hwTs 差 < 此值视为同一帧 |
| `--no-depth` | false | 不采集深度流 |
| `--no-color` | false | 不采集彩色流 |
| `--csv=PATH` | **必填** | CSV 导出路径 |
| `--help` | — | 显示帮助 |

## 输出解读

### 控制台报告

```
=== DataCollector: Start ===
Found 2 device(s)
Device 0: SN=xxx  Name=Gemini 305g  ...
Device 1: SN=yyy  Name=Gemini 305g  ...
Devices: 2  Pairs to check: 1
...

--- 1. Cross-Stream (Depth vs Color) ---
Device 0:
  Pair count: 898
  HW Timestamp Diff:
    Min=-5us  Max=8us  Mean=1.2us  Stddev=3.1us
  System Timestamp Diff:
    Min=-350us  Max=1200us  Mean=15.3us  Stddev=45.7us

--- 2. Cross-Device Depth ---
Device 0 vs Device 1:
  Pair count: 897
  HW Timestamp Diff:
    Min=0us  Max=18000us  Mean=3200.5us  Stddev=2100.3us
  System Timestamp Diff:
    Min=0us  Max=25000us  Mean=5100.2us  Stddev=2800.7us

--- 3. Cross-Device Color ---
Device 0 vs Device 1:
  ...
```

### 关键指标

| 指标 | 含义 |
|------|------|
| **HW Timestamp Diff** | 设备硬件时间戳差，反映**真正的硬件同步精度**（帧曝光时刻的对齐程度） |
| **System Timestamp Diff** | 软件时间戳差，即帧到达主机的时间差，包含传输延迟抖动 |

### CSV 格式

```csv
comparison_type,device_i,device_j,stream,hw_diff_us,sys_diff_us
cross_stream,0,0,depth+color,-3,120
cross_stream,0,0,depth+color,5,350
cross_device,0,1,depth,500,18000
cross_device,0,1,depth,320,15300
cross_device,0,1,color,480,19200
```

每行是一对匹配帧的原始差值（微秒），`hw_diff_us` 为设备时间戳差，`sys_diff_us` 为软件时间戳差。

### Python 可视化输出

```
charts/
├── cross_stream_sync.png      # 同设备跨流：hwTs 差 vs sysTs 差箱型图
├── cross_device_depth.png     # 跨设备 Depth：各 pair 的时间戳差分布
├── cross_device_color.png     # 跨设备 Color：同上
└── summary.html               # HTML 汇总页（统计表格 + 嵌入图表）
```

## 典型测试流程

```bash
# 1. 编译
cd build && cmake .. && make -j$(nproc)

# 2. 测试 30fps
./timestamp_sync_check --fps=30 --duration=30 --csv=result_30fps.csv

# 3. 测试 15fps
./timestamp_sync_check --fps=15 --duration=30 --csv=result_15fps.csv

# 4. 测试 5fps
./timestamp_sync_check --fps=5 --duration=30 --csv=result_5fps.csv

# 5. 生成可视化报告
mkdir -p charts
python src/visualize_sync.py result_30fps.csv --output charts/30fps
python src/visualize_sync.py result_15fps.csv --output charts/15fps
python src/visualize_sync.py result_5fps.csv --output charts/5fps
```