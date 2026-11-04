# Pango FPGA Arbitrary-Angle Distortion Correction

基于紫光同创 Logos-2 **PG2L100H-6IFBG676** 的实时图像畸变矫正工程（2026 全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道 · 选题二）。

本项目在盘古 100Pro+ 开发板上实现 **1280×720@30fps** 的畸变矫正链路：OV5640 DVP 采集 → DDR3 帧缓存 → Brown-Conrady 反向映射 → 4-bank 行缓存 + 双线性重采样 → OSD → HDMI。高阶部分包含 JY61P IMU 仿射姿态补偿、5 档自适应 Gamma，以及高阶任务二的 HIT/MISS/BLK 性能面板。

> **重要：本仓库不包含厂商 IP 和厂商 demo 框架。**
> 完整工程依赖 Pango PDS 生成的 IP（`ipcore/`、`source/rtl/DDR3_50H`）以及 Meyesemi 的 `HDMI_DDR3_OV5640_100H` demo 框架（`cmos_8_16bit.v`、`fram_buf.v`、`sync_vg.v`、`ms72xx_ctl.v` 等）。这些文件带有厂商 proprietary 声明，不能公开转载。本仓库只包含作者编写的算法模块、测试、约束和文档；使用者需自行从板卡厂商获取官方 demo 和 PDS，并按厂商许可使用。详见 [NOTICE.md](NOTICE.md)。

## 功能

- **基础任务一**：OV5640 DVP 8bit 采集、DDR3 帧缓存、HDMI 720P30 显示。
- **基础任务二**：Brown-Conrady 径向（k1/k2）+ 切向（p1/p2）畸变模型，反向映射，Q12.4 源坐标（1/16 像素）。
- **基础任务三**：128 行 4-bank 行缓存，一拍读出 2×2 邻域；双线性插值，`NEAREST` 参数可旁路做 A/B。
- **高阶任务一**：JY61P 六轴 IMU + 仿射补偿（roll/yaw/pitch），OLED/HDMI 角度 OSD；受 128 行缓存限制，有效 roll 约 ±3.5°，更大角度预留 tile 扩展。
- **高阶任务二**：13 级流水线 + 性能面板（OUTPUT FPS、HIT/MISS/BLK、LAT 13 CLK）。
- **高阶任务三**：5 档自适应 Gamma（NIGHT/DIM/NORMAL/BRIGHT/GLARE），阈值 50/90/135/170、高光占比 5%，屏幕显示 Y/G 判档读数。

## 目录

```text
rtl/           作者编写的 RTL
  distortion_map.v          Brown-Conrady 映射（7 级流水，Q12.4）
  distortion_line_buffer.v  128 行 4-bank 行缓存
  bilinear_interp.v         双线性插值
  affine_coeff.v            姿态角 -> 仿射系数
  affine_map.v              2×3 仿射映射
  angle_zero_cal.v          IMU 零点标定
  distortion_param_ctrl.v   畸变参数动态控制
  advanced_task3.v          5 档自适应 Gamma
  mission3_label.v          ADVANCED MISSION 2/3 + 性能面板 + 档位读数
  mission_label.v           任务标题 OSD
  status_label.v            OPERATED / NO OPERATED 状态字
  osd_angle.v               YAW/PITCH/ROLL OSD
imu/            JY61P UART 解析、SSD1306 I2C/OLED、字模
sim/            5 个 iverilog testbench
docs/           design_note.md（设计说明）、video_script_task2.md（演示脚本）
tools/          gen_advanced_task3_lut.py（Gamma 曲线生成/回写）
constraints/    hdmi_top.fdc（引脚/时序约束）
gamma_lut_hex.txt
```

## 集成到官方工程

1. 从板卡厂商获取 **Meyesemi HDMI_DDR3_OV5640_100H** 官方 demo（盘古 100Pro+ / FMC-LPC-IO 子卡 / OV5640）和 Pango PDS 2022.2-SP6.4。
2. 在官方 `.pds` 工程中加入 `rtl/`、`imu/`、`sim/` 下的文件；官方顶层 `hdmi_top.v` 需要按本项目的流水线接入（本仓库不包含该顶层，因为它基于厂商 demo）。
3. 数据流：`OV5640 → cmos_8_16bit → fram_buf/DDR3 → affine_map → distortion_map → distortion_line_buffer → bilinear_interp → advanced_task3 → OSD → MS7200`。
4. 合并 `constraints/hdmi_top.fdc` 中的引脚/时序约束；摄像头接 FMC J2，J20=ADJ、J24=2V5。
5. 关键按键（物理丝印）：KEY(C22)=原图/矫正；KEY1(C23)=高阶一；KEY2(B22)=高阶三；KEY3(A22)=高阶二性能面板。

## 仿真

```bash
iverilog -g2012 -o sim_map.vvp rtl/distortion_map.v sim/distortion_map_tb.v && vvp sim_map.vvp
iverilog -g2012 -o sim_lb.vvp rtl/distortion_line_buffer.v sim/distortion_line_buffer_tb.v && vvp sim_lb.vvp
iverilog -g2012 -o sim_bl.vvp rtl/bilinear_interp.v sim/bilinear_interp_tb.v && vvp sim_bl.vvp
iverilog -g2012 -o sim_warp.vvp rtl/distortion_map.v rtl/distortion_line_buffer.v rtl/bilinear_interp.v sim/warp_core_tb.v && vvp sim_warp.vvp
iverilog -g2012 -o sim_adv3.vvp rtl/advanced_task3.v sim/advanced_task3_tb.v && vvp sim_adv3.vvp
```

注意：当前 testbench 有两处已知问题（RTL 本身无问题）：

- `distortion_map_tb` 的 calibrated DUT 需要补齐 `dyn_en`、`k1_dyn`…`p2_dyn`、`in_range` 端口，否则 calibrated 输出为 x、误判 PASS。
- `advanced_task3_tb` 的强光平均亮度期望值写的是 236，但 `CODE_GLARE=0xDF5C` 的实际亮度是 230（RTL 输出 230 正确）；把期望范围改成 226~234 即可。

## 实测结果（最终 21:26 全流程，PDS 2022.2-SP6.4）

| 项目 | 数值 |
|---|---|
| 分辨率/帧率 | 1280×720@30fps |
| LUT | 11,298 / 66,600（16.96%） |
| FF | 9,079 / 133,200（6.82%） |
| DRM | 137 / 155（88.39%） |
| APM | 99 / 240（41.25%） |
| pix_clk WNS | +6.137 ns，All Constraints Met |
| 比特流 | `hdmi_top.sbit`，SHA256 `c17e8ef6a360ec514ed2a06db9dcb2f2b6c6c3b7da02ec72dc4aa507ac8342a5` |

> 比特流未包含在仓库中（包含厂商 IP 配置），仅给出构建结果和 SHA。

## 已知限制

- 128 行行缓存只支持 roll 约 ±3.5°，更大角度需要从 DDR3 分块反向取点的 tile 架构。
- 高阶任务二面板的 `LAT 13 CLK` 是 `DATA_LATENCY=13` 的设计常量，其余帧率/HIT/MISS/BLK 为每帧实时统计。
- 高阶二/三保持标定常量、不跟随 IMU；姿态补偿仅在高阶任务一使能。
- OV5640 默认开启 AEC，强光判据以高光占比 5% 为主；现场可用第二行 G 读数微调 `BR_GLARE`。

## 许可与第三方声明

本仓库中作者编写的代码、测试和文档用于学习、研究与竞赛交流；未附开源许可证。厂商 IP、官方 demo 框架、器件数据手册等第三方内容不在本仓库中，其版权和许可归原厂商所有。使用本仓库代码前请自行确认并遵守 Pango / Meyesemi 的许可条款。
