# libvgm 扩展芯片与基板技术参考文档库

本目录收录了关于 `libvgm` 新增芯片（Ensoniq ES5505 / ES5506 / ES5510、ZOOM ZSG-2、TI TMS57002）及其所属经典街机与合成器基板的完整技术资料。

---

## 目录索引

1. [Ensoniq ES5505 (OTIS) 与 ES5506 (OTTO) 芯片技术规范](ES5505_ES5506_OTIS_OTTO.md)
   * 芯片概述与架构演进（OTIS vs OTTO）
   * 32 通道复音发声单元、相位累加器与 20-bit / 18-bit 样本插值算法
   * 实时低通数字滤波器与回环/双向循环模式
   * 存储器分页（Page Register）、外部 Bank 切换（Taito F3 exbank）
   * VGM 扩展命令规范（0xD5, 0xD6, 0xD7, 0xE3）

2. [Ensoniq ES5510 (ESP) 音频效果 DSP 处理器规范](ES5510_ESP.md)
   * 哈佛架构微处理器、45-bit/48-bit 指令字结构与 160 步微码空间
   * 24-bit 通用寄存器（192 GPR）与 23 个专用寄存器（DIL, DOL, CCR, CMR 等）
   * 24x24-bit 乘法累加器（48-bit MACHL）与 DRAM 环形延时线寻址生成器
   * 主机接口通信协议、ESPHALT 控制线与状态控制
   * VGM 0xD8 指令集与信号流路由

3. [Taito F3 System (Cybercore) 街机音频子系统架构](TAITO_F3_SYSTEM.md)
   * F3 基板整体架构：主 CPU (68EC020) 与音频 CPU (68000 @ 16MHz)
   * 68000 音频子系统内存映射与双口通信协议
   * ES5505 + ES5510 硬件连线、串行音频路由与外部 Bank 切换逻辑
   * 模拟输出混合与测试模式数字音量调控
   * 经典游戏音频实现案例（《Darius Gaiden》、《RayForce》、《Elevator Action Returns》等）

4. [ZOOM ZSG-2 48 通道波表合成器技术参考](ZOOM_ZSG2.md)
   * 48 声道独立发声引擎与 2:1 ADPCM/压缩格式解析（32-bit 块到 4 个 16-bit 样本）
   * 完整寄存器映射（通道特定寄存器 0x000..0x5FE，全局控制寄存器 0x600..0x63E）
   * 音量包络渐变（Volume Ramping）与加重/低通滤波渐变算法
   * Key-On / Key-Off 控制时序与 ROM 回读机制
   * MAME 核心实现细节与通道分配机制

5. [Texas Instruments TMS57002 (DASP) 音频 DSP 处理器参考](TI_TMS57002_DASP.md)
   * 32-bit 专用音频信号处理器内部架构与微指令流水线
   * 存储结构：指令 RAM (PGM: 256x32)、系数 RAM (CMEM: 256x24/32)、数据 RAM (DMEM0/1) 与外部 256KB 延时线
   * 主机接口协议：PLOAD (程序装载)、CLOAD (系数装载) 与 16 项参数更新队列（Address-matched Update Queue）
   * 4 通道串行音频输入输出（SI0..SI3, SO0..SO3）与 16/24 位数据格式
   * 完整 DSP 状态序列化格式（VGM 数据块 0x98）

6. [Taito Zoom 音频系统（FX-1B / Wolf / G-NET / ZN-1 / ZN-2）架构参考](TAITO_ZOOM_SOUND_SYSTEM.md)
   * 音频协处理器：松下 MN10200 16-bit RISC 微控制器子系统
   * ZSG-2 波表芯片与 TMS57002 DSP 的硬件互联与混音拓扑
   * 混音控制寄存器（8.8 定点数增益矩阵）
   * DSP 预运行机制（Pre-run）与瞬时状态恢复
   * VGM 扩展命令 0xE4 规范与代表作品（《Ray Storm》、《Ray Crisis》、《G-Darius》等）
