# M5Stack Atom Matrix (5×5 RGB LED) 串口灯光控制 — 官方文档核实报告

> 状态: ✅ 已核实 (2026-08-17)。固件 API 用法已按 M5Atom 库当前源码验证并修正。
> 相关文件: `cpp_eyetracker/tests/utils/M5Stack/` (test_m5stack.cpp, CMake 构建) + `cpp_eyetracker/utils/M5Stack/M5Stack.ino` (Arduino 固件) + `cpp_eyetracker/cfg/M5Stack.yaml`
> ⚠️ 固件与 C++ 程序必须分目录: Arduino IDE 会编译草图文件夹内的**所有** .cpp/.h 文件, 若 ino 与 test_m5stack.cpp 同目录, Arduino 会尝试编译 opencv 头文件而报错。

## 一、硬件事实 (官方文档)

| 项目 | 值 | 来源 |
|------|-----|------|
| LED 阵列 | 5×5 = 25 颗 WS2812C (RGB 24bit, GRB 字节序) | M5Stack Atom Matrix 产品页 |
| 数据引脚 | G27 (RGB LED on G27) | M5Atom 仓库 README 引脚表 |
| LED 库 | M5Atom (已标记 deprecated, 官方推荐 M5GFX + M5Unified) | M5Atom 仓库 README |
| 推荐最大亮度 | **60** (亚克力面板散热限制; 默认 20) | Atom Matrix 文档 |

## 二、M5Atom 库 API 核实 (当前源码 M5Atom.h / LED_DisPlay.cpp)

### M5.begin
`M5.begin(true, false, true)` = `(SerialEnable, I2CEnable, DisplayEnable)` — 官方示例 LEDSet.ino 的用法, 与固件一致 ✓

### drawpix (关键)
当前版本签名 (M5Atom.h):
```cpp
void drawpix(uint8_t Number, CRGB Color);            // Number: 0..24
void drawpix(uint8_t xpos, uint8_t ypos, CRGB Color); // xpos/ypos: 0..4
```
实现 (LED_DisPlay.cpp):
- `drawpix(Number, ...)` → `_ledbuff[Number] = Color` (越界 Number ≥ NUM_LEDS 直接忽略)
- `drawpix(xpos, ypos, ...)` → `_ledbuff[xpos + ypos*5] = Color` (行优先, 左上角为 0)
- **两个重载都不直接 show()** — 置 `_mode = kAnimation_frush`, 由库的后台 `run()` 任务调用 `FastLED.show()`
- **FastLED 模板**: `FastLED.addLeds<WS2812, DATA_PIN, GRB>(_ledbuff, LEDNumber)` — **GRB 字节序**

### ⚠️ 历史坑 (GitHub issue #5)
早期版本 drawpix 用 `uint32_t` 颜色且按 RGB 直接写入 WS2812 GRB 链 → **红绿互换** (0xf00000 显示为绿色)。当前库已改为 `FastLED.addLeds<..., GRB>` + CRGB 重载, 修复此 bug。
**结论**: 固件必须显式构造 `CRGB(r, g, b)` 再传给 drawpix (而不是传 0xRRGGBB 整数), 语义明确且与当前库一致。

### setBrightness
```cpp
brightness = (brightness > 100) ? 100 : brightness;   // 库内钳制 0..100
brightness = (40 * brightness / 100);                 // 映射到 FastLED 范围
FastLED.setBrightness(Brightness);
```
- 输入上限 100 (即实际 FastLED 亮度 40)
- 亮度修改不需要重绘 (但固件调用 drawpix 会触发后台 show, 无副作用)

### LED 索引顺序
`drawpix(Number)` 直接写 `_ledbuff[Number]` (无物理映射表) — 即链序 = 行优先视觉序 (0 = 左上角, 24 = 右下角), 与 PC 端 UI 网格预览一致 ✓

## 三、串口协议 (PC → Atom Matrix, 115200 8N1, 换行结尾)

| 指令 | 说明 |
|------|------|
| `PIX <rrggbb> ×25` | 静态: 设置 25 像素 (行优先, 索引 0..24) |
| `ALL <rrggbb>` | 静态: 全部同色 |
| `MODE ALL <rrggbb>` | 静态: 25 颗全亮 (状态灯) |
| `MODE BREATH <rrggbb>` | 动画: 十字呼吸灯 (外臂+中心=呼吸色, 内3×3小十字=白, 正弦亮度 25ms/帧) |
| `MODE FLOW` | 动画: 45° 斜向彩色流转 (每条反对角线 x+y 一个色相, 偏移 40°, 30ms/帧) |
| `CLEAR` | 熄灭 (退出动画) |
| `BRIGHT <0-100>` | 全局亮度 (库钳制上限 100) |

### capture_with_LED 状态 → LED 图案映射 (test_m5stack 按 s 循环发送)

| 状态 | 图案 | 颜色 (RGB) | 指令 |
|------|------|-----------|------|
| PIPER_INIT | 25 全亮 | **蓝 0000ff** | `MODE ALL 0000ff` |
| READY | **十字呼吸** | 绿 00ff00 | `MODE BREATH 00ff00` |
| CAPTURING | **十字呼吸** | 绿 00ff00 | `MODE BREATH 00ff00` |
| WAITING | 25 全亮 | **橙 ff8000** (与绿区分度大) | `MODE ALL ff8000` |
| EXHAUSTED | 25 全亮 | **红 ff0000** | `MODE ALL ff0000` |
| OVER | **横向彩色流转** | 彩虹 (每列偏移 72°) | `MODE FLOW` |

(颜色与 capture_with_LED.cpp 的 drawLedIndicator 同步一致; 后续集成时按此表在 capture_with_LED 中发送)

### 呼吸十字规格 (MODE BREATH)

- 十字 9 颗: 中心行 (10..14) + 中心列 (2,7,17,22), **9 颗亮度相同**
- **配色**: 外臂 (2,10,14,22) + 中心 (12) = **白色**; **内 3×3 小十字 (7,11,13,17) = 呼吸色** (READY 状态)
- 亮度 = 正弦呼吸, **最低约 1/4 (63.75..255)**, 25ms/帧 (白色与呼吸色同步缩放)
- 固件与 PC 预览一致

### 彩流规格 (MODE FLOW)

- 45° 斜向: 每条反对角线 (x+y = 0..8, 共 9 条) 一个色相 = (相位 + (x+y)×40°) % 360
- 30ms/帧, 相位每帧 +8°, 颜色沿 45° 对角线方向流动

PC 端 (test_m5stack.cpp): Win32 `CreateFileA("\\\\.\\COMx")` + DCB 8N1 + 超时; `t` 切换 upper/lower COM 口, `s` 发送随机 HSV 颜色; 5×5 网格 UI 预览。

## 四、固件修正记录 (已应用到 M5Stack.ino)

1. drawpix 显式 `CRGB(r, g, b)` 构造 (规避历史 R/G 互换, 与当前 GRB 模板一致)
2. BRIGHT 钳制改为 0..100 (对齐库内部钳制; 文档建议 ≤60)
3. 注释注明依赖: M5Atom 库 (含 issue #5 修复的版本, 即库管理器当前版)

## 五、烧录步骤 (Arduino IDE)

1. **库**: Library Manager 搜索 `M5Atom` → 安装 "M5Atom by M5Stack" (依赖 FastLED 自动安装)
2. **板卡包**: Boards Manager 搜索 `M5Stack` → 安装 "M5Stack by M5Stack" (esp32 分支)
3. **板卡选择**: Tools → Board → M5Stack → Atom Matrix
4. **打开工程**: File → Open → `cpp_eyetracker/utils/M5Stack/M5Stack.ino`
   (Arduino 要求文件夹名 = .ino 文件名, 当前 `M5Stack/M5Stack.ino` 已满足)
5. **上传**: 选择 COM 口 → Upload (失败可按住复位键重试)

> **为什么 ino 必须单独放 utils/M5Stack**: Arduino IDE 会把草图文件夹内所有 .cpp/.h 当作草图源码编译。
> 若与 test_m5stack.cpp (依赖 opencv/Pylon 等) 同目录, Arduino 会报 `opencv2/opencv.hpp not found`。
> 因此固件 (utils/M5Stack) 与 Windows 控制程序 (tests/utils/M5Stack, CMake 构建) 分目录存放。

## 六、已知限制 / 后续事项

- M5Atom 库已 deprecated (官方推荐 M5Unified + M5GFX) — 当前固件继续用 M5Atom (API 已验证, 库管理器可装); 未来迁移 M5Unified 时协议不变, 仅改 LED 写入层
- 亮度 >60 不建议长时间使用 (面板散热)
- 固件当前不向 PC 回传任何数据 (单向控制)

## 参考来源

- M5Atom 仓库 (README + 源码): https://github.com/m5stack/M5Atom
- M5Atom.h (drawpix 声明): https://github.com/m5stack/M5Atom/blob/master/src/M5Atom.h
- LED_DisPlay.cpp (FastLED GRB 模板 / drawpix / setBrightness): https://github.com/m5stack/M5Atom/blob/master/src/utility/LED_DisPlay.cpp
- R/G 互换 bug 讨论: https://github.com/m5stack/M5Atom/issues/5
- Atom Matrix 产品文档 (LED 阵列/亮度建议): https://docs.m5stack.com/en/atom/atom_matrix
- 官方示例 LEDSet.ino (M5.begin 用法): https://github.com/m5stack/M5Atom/blob/f6c492ac301feb22a65d6c74dae67126801a6478/examples/Basics/LEDSet/LedSet.ino
