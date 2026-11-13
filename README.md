# RoboMaster 英雄机器人 · HERO v2.1

## 项目背景

HERO v2.1 工程由 2021 步兵电控框架演进而来，保留了基于 STM32F427II 和 RT-Thread Nano 的整车工程，后来又拆出了 C 型板底盘和 A 型板俯仰实验。原版、队友交付版及整理后的开发目录各有用途。

## 项目目的

本项目用 Git 保存和交接团队资料，允许收录开发中和待验证的工作。功能状态以各工作线的源码、测试和实测记录为准。接手者可以按板型找到工程，在电脑上重新编译和仿真，并与迁移前基线比较。当前整理以保持已有行为、保全交付历史为目标，验收范围为离线编译和仿真。每项结论都注明对应工程及验证层级。

首次获取项目：

```powershell
git clone --branch release/chassis https://github.com/r4in1st-crypto/RoboMaster_HeroRobot.git
cd RoboMaster_HeroRobot
```

之后在本分支工作区运行 `git pull --ff-only` 获取团队更新。

## 项目结构

| 位置 | 内容 |
| --- | --- |
| [src/Hero/chassis](src/Hero/chassis/) | 固件、启动文件、链接脚本、主机测试和 ARM 构建入口 |
| [src/Simulator/chassis](src/Simulator/chassis/) | 对应版本的模型、场景运行脚本、报告与独立复核 |
| [tools/stlink_monitor](tools/stlink_monitor/) | 通用 OpenOCD 读取模块及配套 SWD 配置 |
| [LICENCE](LICENCE) | 随原工程交付的许可文件 |
| [logs](logs/logs_index.md) · [handoffs](handoffs/handoffs_index.md) | 工作记录、复核证据和下一步交接。开发约定见 [AGENTS.md](AGENTS.md)。 |

本地 main 另存原整车 Keil 工程、A 板俯仰资料、架构图与 Legacy 归档。GitHub 默认分支 release/chassis 提供两套底盘工程及其配套仿真。

## 调试方法

在仓库根目录运行对应版本的主机测试：

```powershell
./src/Hero/chassis/type_c_dr16/run_host_tests.ps1
./src/Hero/chassis/type_c_dr16_direct/run_host_tests.ps1
```

**底盘行为与迁移回归。** 先按版本运行主机测试，再运行对应仿真与独立复核。异常时先看本轮的编译日志、`runner.log` 和 `summary.json`，再用 `raw.csv` 定位场景和时间点。历史版独立复核使用 `-LeftVerticalPolicy ignore`。比较固件时对同版本 BIN 计算 SHA-256，与[迁移前基线](logs/20261113_172519_log.md)核对。

<details>
<summary>展开两套底盘的离线构建、仿真和复核命令（PowerShell）</summary>

从仓库根目录运行。以下变量供两套入口共用，`$tests = (Resolve-Path src/Hero/chassis).Path`，`$zig` 指向 Zig 可执行文件，`$armBin` 指向 GNU Arm 工具链。每次运行指定新的绝对输出目录 `$out`、任务标识 `$job` 和 `$nonce`。独立复核报告写在运行目录之外。

```powershell
$s = Join-Path $tests 'type_c_dr16'
& "$s/build.ps1" -ToolchainBin $armBin
& "./src/Simulator/chassis/type_c_dr16/run_simulation.ps1" -SourceDir $s -CompilerPath $zig -OutputDir $out -JobId $job -Nonce $nonce -LeftVerticalPolicy ignore
python -B "./src/Simulator/chassis/type_c_dr16/independent_review.py" --preflight --output-dir $out --job-id $job --nonce $nonce --report "${out}_review.json"
```

direct 版使用另一组输出目录和运行标识。

```powershell
$s = Join-Path $tests 'type_c_dr16_direct'
& "$s/build.ps1" -ToolchainBin $armBin
python -B "./src/Simulator/chassis/type_c_dr16_direct/run_prebuilt.py" --compile-from $s --compiler $zig --output-dir $out --job-id $job --nonce $nonce
python -B "./src/Simulator/chassis/type_c_dr16_direct/independent_review.py" --preflight --output-dir $out --job-id $job --nonce $nonce --report "${out}_review.json"
```

</details>

## 工具依赖

| 用途 | 工具与版本 |
| --- | --- |
| C 板固件构建 | Windows PowerShell 环境，本轮用 PowerShell 7.6.10；GNU Tools for STM32 GCC 14.3.1，需 gcc、objcopy、size、nm。本机工具目录为 `%LOCALAPPDATA%/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin`。 |
| 主机测试和仿真 | Zig 0.13.0；Python 3.11+，已验证 3.14.6。Zig 可加入 PATH 或显式传入可执行文件路径。 |
| 通用 SWD 读取模块 | 实物观测另需 OpenOCD 与探针驱动。本分支提供读取模块和配置。 |
| 本地整车工程 | Keil MDK，工程选择 ARMCC 5.06 update 5，迁移后构建待验证。 |

## 更新日志摘要

| 日期 | 变更 |
| --- | --- |
| 2026-11-13 | 整车、底盘、A 板和历史归档的状态已汇总到本页。队友从[最近交接](handoffs/handoffs_index.md)接续。 |
| 2026-11-13 | 两套底盘完成目录拆分，迁移前后 BIN 哈希一致，发布底盘源码、仿真和通用读取模块。[工作记录与证据](logs/20261113_172519_log.md) |

## 项目总进度

| 工作线 | 已完成 | 待完成或确认 |
| --- | --- | --- |
| C 板底盘 | 两版主机测试通过，迁移前后 BIN 分别一致。历史版 14 场景、direct 版 7 场景均通过独立复核。 | 当前结论覆盖离线迁移验收；原有长期实车问题仍保留在交付记录中。 |
| 原整车 Keil 工程 | 70 个工程文件引用位置及哈希已核对。 | 迁移后编译待验证；原工程的一个头文件搜索目录缺失。 |
| 原整车发射与俯仰联动 | 源码中有发射任务启动和控制函数实现。 | 联动完成情况待核实；本轮没有整车联调或实车验证记录。 |
| A 板 Pitch | 工程索引已建立，两套主机测试通过。 | 完整固件仍在交付树中，尚无本轮目标构建结果。 |
| 监测工具 | 新目录离线回归 10 项通过，候选副本中亦已复跑。 | 本轮验证范围为主机侧。 |
| 历史归档 | 本地已建立原件与增量基线，并验证可还原队友交付的 34,011 个文件。 | 完整归档保存在本机；远端全新克隆的底盘回归待完成。 |
