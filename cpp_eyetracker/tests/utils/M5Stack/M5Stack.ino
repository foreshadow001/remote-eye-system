// ================== M5Stack.ino ==================
// M5Stack Atom Matrix (5x5 RGB LED) 串口控制固件
// 配合 tests/utils/M5Stack/test_m5stack.cpp 使用 (115200, 8N1)
//
// 协议 (ASCII, 每行以 \n 结尾):
//   PIX <rrggbb> <rrggbb> ... (25 个, 行优先索引 0..24)  — 设置 25 个像素
//   ALL <rrggbb>                                         — 全部同色
//   CLEAR                                                — 熄灭
//   BRIGHT <0-100>                                       — 全局亮度 (库内部钳制上限 100, 建议 <=60)
//
// 依赖: M5Atom 库 (Arduino IDE 库管理器搜索 "M5Atom"; 需包含 issue #5 修复的当前版本)
// 烧录: 选择 M5Atom 开发板, 115200 上传
// API 核实详见 plan/m5stack_atom_matrix.md

#include "M5Atom.h"

#define NUM_LEDS 25

uint32_t g_pixels[NUM_LEDS];   // 0xRRGGBB 存储, 发送时显式拆分为 CRGB

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
    if (cmd.startsWith("PIX ")) {
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
        uint32_t c = parseHex(tok);
        for (int i = 0; i < NUM_LEDS; i++) g_pixels[i] = c;
        applyPixels();
    } else if (cmd == "CLEAR") {
        for (int i = 0; i < NUM_LEDS; i++) g_pixels[i] = 0;
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
    delay(1);
}
