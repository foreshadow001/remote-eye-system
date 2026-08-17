// ================== M5Stack.ino ==================
// M5Stack Atom Matrix (5x5 RGB LED) 串口控制固件
// 配合 tests/utils/M5Stack/test_m5stack.cpp 使用 (115200, 8N1)
//
// 协议 (ASCII, 每行以 \n 结尾):
//   PIX <rrggbb> <rrggbb> ... (25 个, 行优先索引 0..24)  — 静态: 设置 25 个像素
//   ALL <rrggbb>                                         — 静态: 全部同色
//   MODE ALL <rrggbb>                                    — 静态: 25 个全亮 (状态灯)
//   MODE BREATH <rrggbb>                                 — 动画: 十字呼吸灯 (中心行+列, 9 颗)
//   MODE FLOW                                            — 动画: 横向彩色流转 (每列不同色相, 彩虹循环)
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

// 十字 = 中心行 (10..14) + 中心列 (2,7,17,22), 共 9 颗
// 权重: 中心 1.0 / 邻位 0.7 / 外缘 0.4 (越中心越亮), ×255 定点
const uint8_t CROSS_IDX[9] = {2, 7, 10, 11, 12, 13, 14, 17, 22};
const uint8_t CROSS_W[9]   = {102, 179, 102, 179, 255, 179, 102, 179, 102};

// 返回像素 i 的十字权重 (0 = 不在十字上)
uint8_t crossWeight(uint8_t i) {
    for (uint8_t j = 0; j < 9; j++) if (CROSS_IDX[j] == i) return CROSS_W[j];
    return 0;
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
        // 十字呼吸动画: 亮度正弦波动, 最低约 50% (127.5..255), 越中心越亮 (权重缩放)
        g_phase = (g_phase + 6) % 360;
        uint8_t b = (uint8_t)(191.25 + 63.75 * sin(g_phase * PI / 180.0));
        for (int i = 0; i < NUM_LEDS; i++) {
            uint8_t w = crossWeight(i);
            if (w == 0) { M5.dis.drawpix(i, CRGB(0, 0, 0)); continue; }
            uint8_t bb = (uint8_t)((uint16_t)b * w / 255);
            uint8_t r = (uint8_t)((uint16_t)((g_breath_color >> 16) & 0xFF) * bb / 255);
            uint8_t g = (uint8_t)((uint16_t)((g_breath_color >> 8) & 0xFF) * bb / 255);
            uint8_t bl = (uint8_t)((uint16_t)(g_breath_color & 0xFF) * bb / 255);
            M5.dis.drawpix(i, CRGB(r, g, bl));
        }
        delay(25);
    } else if (g_mode == 2) {
        // 横向彩色流转: 每列色相 = 相位 + 列偏移 (72°), 随时间向右流动
        g_phase = (g_phase + 8) % 360;
        for (int x = 0; x < 5; x++) {
            uint16_t hue = (g_phase + x * 72) % 360;
            CRGB c = CRGB(CHSV(hue, 255, 255));   // FastLED HSV→RGB
            for (int y = 0; y < 5; y++) M5.dis.drawpix(x + y * 5, c);
        }
        delay(30);
    } else {
        delay(1);
    }
}
