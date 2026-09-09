# Jetson AGX Orin 系统备份指南

## 背景

| 项目 | 情况 |
|------|------|
| 板子型号 | NVIDIA Jetson AGX Orin Developer Kit |
| 架构 | aarch64（ARM64） |
| 系统 | Ubuntu（根分区在 eMMC `/dev/mmcblk0`） |
| 启动盘 | 焊死的 eMMC，59G，15 个分区 + boot0/boot1 引导分区 |
| 板上额外硬盘 | 476G NVMe SSD（`/dev/nvme0n1`，全新空盘） |
| 备份仓库 | i9 主机（Windows 11），只存镜像，不运行板子系统 |

> 目标：把板子整个系统做成镜像，存到 i9 上。板子系统损坏后，从 i9 把镜像写回 eMMC 恢复。

---

## 一、格式化并挂载板上 NVMe

```bash
# 格式化（仅对空盘执行，会清空盘上数据）
sudo mkfs.ext4 /dev/nvme0n1

# 创建挂载点并挂载
sudo mkdir -p /mnt/backup
sudo mount /dev/nvme0n1 /mnt/backup

# 确认挂载成功，应显示约 469G 可用
df -h /mnt/backup
```

---

## 二、整盘镜像（三条命令缺一不可）

```bash
# 1) 主盘镜像（59G，约 3~6 分钟，注意看进度条）
sudo dd if=/dev/mmcblk0 of=/mnt/backup/jetson_emmc_$(date +%Y%m%d).img bs=4M status=progress conv=fsync

# 2) 硬件引导分区 boot0（4M，引导程序所在，漏了恢复不了）
sudo dd if=/dev/mmcblk0boot0 of=/mnt/backup/boot0.img bs=4M conv=fsync

# 3) 硬件引导分区 boot1（4M）
sudo dd if=/dev/mmcblk0boot1 of=/mnt/backup/boot1.img bs=4M conv=fsync
```

---

## 三、校验镜像文件

```bash
ls -lh /mnt/backup/
```

预期结果：

- `jetson_emmc_YYYYMMDD.img` ≈ 59G
- `boot0.img` = 4M
- `boot1.img` = 4M

---

## 四、把镜像拷到 i9

板子先查 IP：`ip a`

在 i9（Windows PowerShell）执行：

```powershell
scp user@<板子IP>:/mnt/backup/jetson_emmc_*.img .
scp user@<板子IP>:/mnt/backup/boot0.img .
scp user@<板子IP>:/mnt/backup/boot1.img .
```

> 提示：也可以把 NVMe 拆下来插到 i9 上用读卡盒拷贝，比 scp 快。
> 镜像约 59G，i9 上至少留 60G 空间。

---

## 五、（可选）压缩省空间

在 i9 上压缩比在板子上快得多，拿到镜像后可在 i9 上执行：

```powershell
# 用 7-Zip 压缩成 .7z 可大幅缩小体积
```

---

## 注意事项

1. **三条 dd 都要执行**，boot0/boot1 是硬件引导分区，只备份主盘无法恢复。
2. **系统运行时 dd 的镜像**，相当于一次非正常关机的状态，恢复后首次启动会跑一次 `fsck` 自动修复，对灾备用完全够用。
3. 若想镜像更干净，可先 `sudo mount -o remount,ro /` 把根分区挂只读，dd 完再 `sudo mount -o remount,rw /` 恢复（运行中的服务可能报错，属正常）。
4. **不要在盘有数据时执行 `mkfs.ext4`**，会清空数据。

---

## 附：板子损坏后如何恢复（写回 eMMC）

### 情况 A：板子还能从 U 盘 / NVMe 启动

启动一个 Linux 后执行：

```bash
sudo dd if=/mnt/backup/jetson_emmc_YYYYMMDD.img of=/dev/mmcblk0 bs=4M status=progress conv=fsync
sudo dd if=/mnt/backup/boot0.img of=/dev/mmcblk0boot0 bs=4M conv=fsync
sudo dd if=/mnt/backup/boot1.img of=/dev/mmcblk0boot1 bs=4M conv=fsync
```

### 情况 B：板子彻底起不来（Recovery 模式刷写）

1. 将 Orin 进入 Recovery 模式（短接 FC_REC 引脚后上电）。
2. 用一台 **Linux 主机**（i9 需装 Ubuntu 或双系统）通过 USB 连接板子。
3. 用 NVIDIA 官方刷机工具 `flash.sh` 把镜像写回。

> 注意：Windows 的 i9 无法直接刷机，Recovery 模式需要 Linux 主机。
