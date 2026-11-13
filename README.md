# RoboMaster HERO · C 板底盘与主机仿真

本分支保存 HERO 四轮底盘的两套独立 STM32F407 工程和各自的主机闭环仿真。历史版和 direct 版分别编译本目录的遥控解析器、轮速控制器及场景程序。

| 位置 | 内容 |
| --- | --- |
| [src/Hero/chassis](src/Hero/chassis/) | 固件、启动文件、链接脚本、主机测试和 ARM 构建入口 |
| [src/Simulator/chassis](src/Simulator/chassis/) | 对应版本的模型、场景运行脚本、报告与独立复核 |
| [tools/stlink_monitor](tools/stlink_monitor/) | 通用 OpenOCD 读取模块及配套 SWD 配置 |
| [LICENCE](LICENCE) | 随原工程交付的许可文件 |

## 离线验证

在仓库根目录通过 PowerShell 运行。安装 Zig 0.13.0 并加入 PATH，Python 使用 3.11 或更高版本。构建 ARM 固件时向对应 build.ps1 传入 -ToolchainBin，指向含 gcc、objcopy、size、nm 的 GNU Arm 工具目录；项目基线使用 GNU Tools for STM32 14.3.1。

```powershell
./src/Hero/chassis/type_c_dr16/run_host_tests.ps1
./src/Hero/chassis/type_c_dr16_direct/run_host_tests.ps1
```

direct 版源码仿真与独立复核示例：

```powershell
$nonce = [guid]::NewGuid().ToString('N')
$out = Join-Path (Get-Location).Path "validation/direct_$nonce"
$sim = './src/Simulator/chassis/type_c_dr16_direct'
python -B "$sim/run_prebuilt.py" --compile-from ./src/Hero/chassis/type_c_dr16_direct --output-dir $out --job-id direct --nonce $nonce
python -B "$sim/independent_review.py" --preflight --output-dir $out --job-id direct --nonce $nonce --report "${out}_review.json"
```

历史版使用同目录层级下 type_c_dr16/run_simulation.ps1，传入新的绝对输出目录、JobId、Nonce 及 -LeftVerticalPolicy ignore 后，再运行该版 independent_review.py。既有软件回归覆盖历史版 14 个场景和 direct 版 7 个场景，模型采用假设的一阶转子响应。
