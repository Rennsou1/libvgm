# Ensoniq ES5505 (OTIS) 与 ES5506 (OTTO) 芯片技术规范

## 1. 概述与芯片历史

**Ensoniq ES5505（研发代号 OTIS - Ensoniq Technology Integrating Synthesis）** 与其第二代演进版 **ES5506（研发代号 OTTO - Ensoniq Technology Translating Operator）** 是由美国 Ensoniq 公司在 1980 年代末至 1990 年代初设计的专用大规模集成电路（VLSI）多声道波表音频合成芯片。

这两颗芯片凭借其高保真度的插值算法、独立的声音通道以及内置数字滤波能力，被广泛应用于专业电子乐器（如 Ensoniq VFX, SD-1, SQ-1, TS-10, ASR-10, EPS-16+ 等）、PC 高级声卡（如 Gravis UltraSound, Ensoniq Soundscape）以及 Taito 的多款 32 位街机基板（如 Taito F3 Cybercore 系统）。

### 主要技术参数对比

| 特性 | ES5505 (OTIS) | ES5506 (OTTO) |
| :--- | :--- | :--- |
| **制造工艺** | 2.0 微米 双层金属 CMOS | 1.5 微米 双层金属 CMOS |
| **封装形式** | 48-pin DIP | 68-pin PLCC |
| **复音频道** | 32 个独立振荡器通道 | 32 个独立振荡器通道 |
| **内部计算精度** | 16-bit 线性精度 | 至少 18-bit 至 20-bit 线性精度 |
| **样本数据格式** | 16-bit 线性 PCM | 16-bit 线性 PCM / 压缩数据模式（$\mu$-law/CMPD） |
| **外部总线接口** | 68000 兼容异步总线 | 68000 兼容异步总线（支持 8/16/32 位操作） |
| **串行输出接口** | 4 对立体声串行通道（8 声道） | 6 对立体声串行通道（12 声道） |
| **A/D 转换接口** | 8 选 1 电位器/滑轮模拟量输入 | 8 选 1 电位器/滑轮模拟量输入 |
| **内置数字滤波** | 实时二阶数字低通滤波器 | 实时可编程数字滤波器 |
| **双芯片级联** | 需外部电路逻辑配合 | 原生支持双 OTTO 主从同步级联 |

---

## 2. 内部架构与发声模型

### 2.1 振荡器与相位累加器
每个振荡器（Voice/Oscillator）维护一个 32 位的相位累加器（Accumulator），由以下核心部分构成：
* **整数地址指针**：指向当前波表样本在存储器中的基地址。
* **小数频率分量**：11 至 16 位的小数累加步长，用于实现连续无级变调（Pitch Pitching）。
* **样条插值引擎**：读取相邻样本并通过累加器的小数部分计算线性插值，有效抑制低采样率变调时产生的镜像混叠噪声。

### 2.2 循环与播放模式（Loop Modes）
每个通道由控制寄存器中的 `BLE`（Bidirectional Loop Enable）、`LPE`（Loop Enable）、`DIR`（Direction）和 `STOP0/STOP1` 位控制：
1. **单次播放（One-shot）**：累加器从 `START` 扫描到 `END`，当触发边界后置位 `STOP` 位并停止发声。
2. **正向循环（Forward Loop）**：当累加器扫描到 `END` 地址时，硬件自动将地址重置到 `START`（或循环起始点），实现无缝循环。
3. **双向回环（Ping-Pong / Bidirectional Loop）**：累加器到达 `END` 时反转扫描方向向 `START` 扫描，到达 `START` 后再次正向扫描。
4. **特殊终止条件**：在 ES5506 中，若 `START == END`，芯片会在下一个采样周期强行将通道置为停止；而 ES5505 在部分合成器固件中允许在此类异常状态下继续运行特定的直流偏置输出。

---

## 3. 寄存器映射架构

ES5505 / ES5506 采用分页寄存器设计（Paged Architecture）。芯片将 32 个复音声道的控制寄存器映射在不同的 Page 中，全局控制寄存器在所有 Page 中均可见。

### 3.1 全局寄存器（Global Across All Pages）
* **0x78 (PAGE)**：页选择寄存器（0x00 ~ 0x1F 选择当前激活声道的参数区）。
* **0x70 (IRQV)**：当前发生中断的声音通道向量号（Interrupt Vector Register）。
* **0x68 (PAR)**：电位器 A/D 采样寄存器（Pot A/D Register）。

### 3.2 声道专用寄存器（0x00 ~ 0x3F，受 PAGE 寄存器寻址）
* **0x00 (CR - Control Register)**：16 位通道控制寄存器：
  * `Bit 15-14 (BS1, BS0)`：存储器 Bank 选择。
  * `Bit 13 (CMPD)`：压缩数据模式使能（仅 ES5506）。
  * `Bit 12-10 (CA2, CA1, CA0)`：输出串行物理通道分配（Channel Assign）。
  * `Bit 9-8 (LP4, LP3)`：低通滤波器工作模式选择。
  * `Bit 7 (IRQ)`：当前声道中断挂起状态标志。
  * `Bit 6 (DIR)`：播放累加器步进方向（0 = 正向递增，1 = 反向递减）。
  * `Bit 5 (IRQE)`：声道中断允许使能。
  * `Bit 4 (BLE)`：双向循环使能。
  * `Bit 3 (LPE)`：循环模式使能。
  * `Bit 2 (LEI)`：忽略循环终点标志。
  * `Bit 1-0 (STOP1, STOP0)`：发声停止与静音控制。
* **0x08 (START)**：样本起始地址寄存器。
* **0x10 (END)**：样本结束地址寄存器。
* **0x18 (ACCUM)**：当前相位累加器数值（包含整数地址与小数步进）。
* **0x20 (FREQ)**：播放频率步进寄存器。
* **0x28 (ENVVAL)**：音量与包络增益设定。

---

## 4. 存储器架构与外部 Bank 机制（Taito F3 exbank）

在街机主板（如 Taito F3 Cybercore）上，由于游戏使用了高达 16MB 甚至 32MB 的大容量样本 ROM，超过了 ES5505 原生 16 位的直接寻址范围。Taito 通过外部 CPLD/定制门阵列设计了 **exbank** 寄存器机制：
* 主 CPU 通过向专用的外部 I/O 端口写入 8 位 Bank 编号，为特定的声音通道动态绑定 1MB~4MB 的样本分页窗口。
* `libvgm` 在实现中通过扩展命令模拟了 4 个独立的 Sample ROM 区域，支持每个区域的独立偏移量、大小映射以及区域别名（Region Aliasing）。

---

## 5. VGM 格式扩展指令规范

为了高精度回放使用 ES5505/ES5506 的 VGM 音频文件，社区制定了以下标准扩展指令：
* **`0xD5 pp aa dd`**：ES5503 寄存器写入（兼容保留）。
* **`0xD6 aa ddee`**：ES5506 标准寄存器写入（16 位大端数据 `ddee`，地址 `aa`）。
* **`0xD7 pp aa dd`**：ES5506 扩展配置命令：
  * `pp` 最高位（bit 7）为芯片编号（0 或 1）。
  * 包含子指令：ROM 区域选择、外部 Bank（exbank）切换、窗口偏移与中断应答。
* **`0xE3 pp cc gggg`**：ES5506 输出声道增益配置（`cc` 为物理声道号，`gggg` 为 8.8 大端定点数增益值）。
