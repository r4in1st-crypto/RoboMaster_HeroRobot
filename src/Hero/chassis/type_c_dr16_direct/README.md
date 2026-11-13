# C 板 DR16 direct · VS Code 工作区

打开本目录或 [Direct-C407.code-workspace](Direct-C407.code-workspace)。在 Windows 上使用 Python 3.11+、PowerShell 7、GNU Arm GCC 和 STM32CubeProgrammer CLI。无需安装 VS Code 烧录扩展。

| 操作 | 入口 |
| --- | --- |
| 全量编译 | 按 Ctrl+Shift+B，默认任务 Rebuild 调用 build.ps1，重新编译所有固件源文件并链接，在 build/ 生成 BIN、HEX、ELF。 |
| 离线检查 | 在“终端 → 运行任务”选择 Check Flash (offline)。先 Rebuild，再检查源码、镜像哈希、启动向量和本机烧录工具路径。 |
| 烧录实板 | ST-LINK 连接目标 C 板后，在“终端 → 运行任务”选择 Flash。任务先 Rebuild，然后检查目标设备 ID 和 1 MiB 容量，使用识别到的探针序列号，双读完整 Flash 备份，写入并校验，再独立回读比对并复位。备份和日志位于 flash_backups/ 的本次运行子目录。 |

构建入口优先选用本机 STM32 工具包的 14.3.1+st.2。其他安装位置可用 HERO_ARM_BIN 和 HERO_PROGRAMMER_BIN 分别指定包含 arm-none-eabi-gcc.exe 和 STM32_Programmer_CLI.exe 的目录，也可通过 PATH 提供工具。

目标为 STM32F407IG 的四轮底盘。上电第一次收到合法遥控帧后，S1 上挡即可驱动，合法的 S1 中/下挡命令令输出归零。检测到遥控错误帧后立即将目标和电流指令置零，保持到下一帧合法数据；新帧 S1 上挡即可按其摇杆值运行。单纯没有新帧仍无超时停机，关闭遥控器不能作为停车操作。首次上板建议架空四轮、保持摇杆中位和 S1 下挡，先核对轮向。编译和离线检查不访问目标板。
