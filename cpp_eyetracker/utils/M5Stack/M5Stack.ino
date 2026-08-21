// ================== M5Stack.ino ==================
// M5Stack Atom Matrix (5x5 RGB LED) 串口控制固件
// 配合 tests/utils/M5Stack/test_m5stack.cpp 使用 (115200, 8N1)
//
// 协议 (ASCII, 每行以 \n 结尾):
//   PIX <rrggbb> <rrggbb> ... (25 个, 行优先索引 0..24)  — 静态: 设置 25 个像素
//   ALL <rrggbb>                                         — 静态: 全部同色
//   MODE ALL <rrggbb>                                    — 静态: 25 个全亮 (状态灯)
//   MODE BREATH <rrggbb>                                 — 动画: 25 颗全亮呼吸灯 (正弦亮度, 最低约 1/4)
//   MODE FLOW                                            — 动画: 45° 斜向彩色流转 (每条反对角线一个色相)
//   CLEAR                                                — 熄灭 (退出动画)
//   BRIGHT <0-100>                                       — 全局亮度 (库内部钳制上限 100, 建议 <=60)
//
// 依赖: M5Atom 库 (Arduino IDE 库管理器搜索 "M5Atom"; 需包含 issue #5 修复的当前版本)
// 烧录: 选择 M5Atom 开发板, 115200 上传
// API 核实详见 plan/m5stack_atom_matrix.md

#include "M5Atom.h"

#define NUM_LEDS 25

uint32_t g_pixels[NUM_LEDS];        // 0xRRGGBB 存储 (静态模式), 发送时显式拆分为 CRGB
uint8_t  g_mode = 0;                // 0 = 静态, 1 = 十字呼吸动画, 2 = 横向彩流动画
uint32_t g_breath_color = 0x00FF00;
uint16_t g_phase = 0;               // 动画相位 (度)

// 十字 = 中心行 (10..14) + 中心列 (2,7,17,22), 共 9 颗 (亮度相同)
// 配色: 外臂+中心 (2,10,12,14,22) = 白色; 内 3x3 小十字 (7,11,13,17) = 呼吸色
const uint8_t CROSS_IDX[9] = {2, 7, 10, 11, 12, 13, 14, 17, 22};

bool isCross(uint8_t i) {
    for (uint8_t j = 0; j < 9; j++) if (CROSS_IDX[j] == i) return true;
    return false;
}

bool isWhitePx(uint8_t i) {
    return (i == 2 || i == 10 || i == 12 || i == 14 || i == 22);
}

void applyPixels() {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t c = g_pixels[i];
        // 显式 CRGB(r,g,b) 构造 (库为 FastLED GRB 模板, 传 0xRRGGBB 整数有历史 R/G 互换坑)
        M5.dis.drawpix(i, CRGB((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF));
    }
}

uint32_t parseHex(const String& tok) {
    uint32_t v = 0;
    for (unsigned int i = 0; i < tok.length() && i < 6; i++) {
        char c = tok[i];
        v <<= 4;
        if (c >= '0' && c <= '9') v |= c - '0';
        else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
    }
    return v;
}

void handleLine(const String& line) {
    String cmd = line;
    cmd.trim();
    if (cmd.startsWith("MODE ALL ")) {
        String tok = cmd.substring(9);
        tok.trim();
        g_mode = 0;
        uint32_t c = parseHex(tok);
        for (int i = 0; i < NUM_LEDS; i++) g_pixels[i] = c;
        applyPixels();
    } else if (cmd.startsWith("MODE BREATH ")) {
        String tok = cmd.substring(12);
        tok.trim();
        g_mode = 1;
        g_breath_color = parseHex(tok);
        g_phase = 0;
    } else if (cmd == "MODE FLOW") {
        g_mode = 2;
        g_phase = 0;
    } else if (cmd == "CLEAR") {
        g_mode = 0;
        for (int i = 0; i < NUM_LEDS; i++) g_pixels[i] = 0;
        applyPixels();
    } else if (cmd.startsWith("PIX ")) {
        g_mode = 0;   // 退出动画模式 (BREATH/FLOW 循环会每帧覆盖静态像素)
        String payload = cmd.substring(4);
        int idx = 0;
        int pos = 0;
        while (idx < NUM_LEDS) {
            int sp = payload.indexOf(' ', pos);
            String tok = (sp < 0) ? payload.substring(pos) : payload.substring(pos, sp);
            tok.trim();
            if (tok.length() >= 6) g_pixels[idx] = parseHex(tok);
            idx++;
            if (sp < 0) break;
            pos = sp + 1;
        }
        applyPixels();
    } else if (cmd.startsWith("ALL ")) {
        String tok = cmd.substring(4);
        tok.trim();
        g_mode = 0;
        uint32_t c = parseHex(tok);
        for (int i = 0; i < NUM_LEDS; i++) g_pixels[i] = c;
        applyPixels();
    } else if (cmd.startsWith("BRIGHT ")) {
        int b = cmd.substring(7).toInt();
        if (b < 0) b = 0;
        if (b > 100) b = 100;   // 库内部钳制上限 100 (FastLED 40); 文档建议 <=60
        M5.dis.setBrightness(b);
        applyPixels();
    }
}

String g_line = "";

void setup() {
    M5.begin(true, false, true);   // Serial 开启, I2C/外部显示关闭
    M5.dis.setBrightness(20);
    applyPixels();                 // 上电熄灭
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_line.length() > 0) handleLine(g_line);
            g_line = "";
        } else if (g_line.length() < 256) {
            g_line += c;
        }
    }
    if (g_mode == 1) {
        // 全矩阵呼吸动画: 25 颗同色, 正弦波动最低约 1/4 (63.75..255)
        g_phase = (g_phase + 6) % 360;
        uint8_t b = (uint8_t)(159.375 + 95.625 * sin(g_phase * PI / 180.0));
        uint32_t col = g_breath_color;
        uint8_t r = (uint8_t)((uint16_t)((col >> 16) & 0xFF) * b / 255);
        uint8_t g = (uint8_t)((uint16_t)((col >> 8) & 0xFF) * b / 255);
        uint8_t bl = (uint8_t)((uint16_t)(col & 0xFF) * b / 255);
        for (int i = 0; i < NUM_LEDS; i++) M5.dis.drawpix(i, CRGB(r, g, bl));
        delay(25);
    } else if (g_mode == 2) {
        // 45° 斜向彩色流转: 每条反对角线 (x+y 相同) 一个色相, 9 条线 × 40°
        g_phase = (g_phase + 8) % 360;
        for (int y = 0; y < 5; y++) {
            for (int x = 0; x < 5; x++) {
                uint16_t hue = (g_phase + (x + y) * 40) % 360;
                M5.dis.drawpix(x + y * 5, CRGB(CHSV(hue, 255, 255)));   // FastLED HSV→RGB
            }
        }
        delay(30);
    } else {
        delay(1);
    }
}
