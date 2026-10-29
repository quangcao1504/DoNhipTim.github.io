/*
  Do HR + SpO2 bang MAX30102, hien thi len LCD 1602 I2C (PCF8574)
  ESP32 DevKit + ESP-IDF v5.2 tro len (driver i2c_master moi)
  CHI DUNG HOC TAP - KHONG PHAI THIET BI Y TE

  Noi day (I2C chung):
    MAX30102 VIN -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22
    LCD1602  VCC -> 3V3 (hoac 5V neu da xu ly pull-up), GND -> GND,
             SDA -> GPIO21, SCL -> GPIO22
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "HR_SPO2";

/* ---------------- Cau hinh ---------------- */
#define I2C_SDA_IO        21
#define I2C_SCL_IO        22
#define MAX_ADDR          0x57
#define LCD_ADDR_1        0x27
#define LCD_ADDR_2        0x3F

#define FS_HZ             25      /* 100 sps / trung binh 4 mau = 25 Hz */
#define WIN_LEN           150     /* 6 giay */
#define STEP              25      /* cap nhat moi ~1 giay */
#define FINGER_IR_MIN     30000.0f
#define MIN_PEAK_DIST     8       /* ~0.32 s => toi da ~187 bpm */

/* ---------------- Thanh ghi MAX30102 ---------------- */
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C   /* Red */
#define REG_LED2_PA       0x0D   /* IR  */
#define REG_PART_ID       0xFF

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_max;
static i2c_master_dev_handle_t s_lcd;

/* =================== MAX30102 =================== */
static esp_err_t max_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(s_max, b, 2, 100);
}

static esp_err_t max_read_reg(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(s_max, &reg, 1, dst, len, 100);
}

static esp_err_t max_init(void)
{
    uint8_t id = 0;
    if (max_read_reg(REG_PART_ID, &id, 1) != ESP_OK || id != 0x15) {
        ESP_LOGE(TAG, "Khong thay MAX30102 (part ID=0x%02X)", id);
        return ESP_FAIL;
    }
    max_write(REG_MODE_CONFIG, 0x40);          /* reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    max_write(REG_FIFO_CONFIG, 0x5F);          /* trung binh 4, rollover, almost-full 15 */
    max_write(REG_SPO2_CONFIG, 0x2F);          /* ADC 4096 nA, 100 sps, 411 us (18 bit) */
    max_write(REG_LED1_PA, 0x3C);              /* Red ~12 mA */
    max_write(REG_LED2_PA, 0x3C);              /* IR  ~12 mA */
    max_write(REG_FIFO_WR_PTR, 0x00);
    max_write(REG_OVF_COUNTER, 0x00);
    max_write(REG_FIFO_RD_PTR, 0x00);
    max_write(REG_MODE_CONFIG, 0x03);          /* che do SpO2: Red + IR */
    return ESP_OK;
}

/* Doc cac mau co trong FIFO. Tra ve so mau, hoac -1 neu loi. */
static int max_read_samples(uint32_t *red, uint32_t *ir, int max_n)
{
    uint8_t p[3];
    if (max_read_reg(REG_FIFO_WR_PTR, p, 3) != ESP_OK) return -1;
    int n = (p[0] - p[2]) & 0x1F;
    if (p[1] != 0) n = 32;                      /* tran FIFO */
    if (n > max_n) n = max_n;
    if (n <= 0) return 0;

    uint8_t buf[6 * 32];
    if (max_read_reg(REG_FIFO_DATA, buf, 6 * n) != ESP_OK) return -1;
    for (int k = 0; k < n; k++) {
        const uint8_t *b = &buf[6 * k];
        red[k] = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
        ir[k]  = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
    }
    return n;
}

/* =================== OLED SSD1306 128x64 (I2C) =================== */
/* Font 5x7, ky tu tu 0x20 (space) den 'z' */
static const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
    0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
    0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
    0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
    0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
    0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
    0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
    0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
    0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
    0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
    0x63,0x14,0x08,0x14,0x63, 0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
    0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
    0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
    0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
    0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
    0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
    0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44,
};

static void oled_cmd(uint8_t c)
{
    uint8_t b[2] = {0x00, c};
    i2c_master_transmit(s_lcd, b, 2, 50);
}

/* Ghi 128 byte du lieu vao 1 page (8 dong diem) */
static void oled_page(uint8_t page, const uint8_t *data)
{
    oled_cmd(0xB0 | page);
    oled_cmd(0x00);
    oled_cmd(0x10);
    uint8_t buf[129];
    buf[0] = 0x40;
    memcpy(&buf[1], data, 128);
    i2c_master_transmit(s_lcd, buf, 129, 100);
}

static void lcd_init(void)
{
    if (!s_lcd) return;
    vTaskDelay(pdMS_TO_TICKS(100));
    static const uint8_t init[] = {
        0xAE, 0xD5,0x80, 0xA8,0x3F, 0xD3,0x00, 0x40, 0x8D,0x14,
        0x20,0x02,              /* page addressing */
        0xA1, 0xC8, 0xDA,0x12, 0x81,0xCF, 0xD9,0xF1, 0xDB,0x40,
        0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof init; i++) oled_cmd(init[i]);

    uint8_t zero[128] = {0};
    for (uint8_t p = 0; p < 8; p++) oled_page(p, zero);
}

/* In 1 dong (toi da 16 ky tu), chieu cao gap doi. row 0 hoac 1 */
static void lcd_row(int row, const char *s)
{
    if (!s_lcd) return;
    uint8_t lo[128] = {0}, hi[128] = {0};
    int x = 0;
    for (int i = 0; i < 16 && s[i]; i++) {
        int c = (uint8_t)s[i];
        if (c < 32 || c > 'z') c = ' ';
        for (int col = 0; col < 5; col++) {
            uint8_t g = font5x7[(c - 32) * 5 + col];
            uint16_t e = 0;
            for (int b = 0; b < 7; b++) if (g & (1 << b)) e |= (3 << (2 * b));
            lo[x] = e & 0xFF;
            hi[x] = e >> 8;
            x++;
        }
        x++;   /* khoang trang giua 2 ky tu */
    }
    uint8_t p = row ? 4 : 1;
    oled_page(p, lo);
    oled_page(p + 1, hi);
}

/* =================== Thuat toan HR + SpO2 =================== */
typedef struct {
    bool  finger;
    bool  hr_ok;
    float hr;
    bool  spo2_ok;
    float spo2;
} result_t;

/* ac[i] = TB 4 mau (loc thap) - TB 25 mau (bo nen cham), i = 24..N-1 */
static void make_ac(const float *x, float *ac)
{
    for (int i = 24; i < WIN_LEN; i++) {
        float lp = 0, bl = 0;
        for (int k = 0; k < 4; k++)  lp += x[i - k];
        for (int k = 0; k < 25; k++) bl += x[i - k];
        ac[i] = lp / 4.0f - bl / 25.0f;
    }
}

static result_t analyze(const float *ir, const float *red)
{
    result_t r = {0};
    float dc_ir = 0, dc_red = 0;
    for (int i = 0; i < WIN_LEN; i++) { dc_ir += ir[i]; dc_red += red[i]; }
    dc_ir /= WIN_LEN; dc_red /= WIN_LEN;

    if (dc_ir < FINGER_IR_MIN) return r;        /* chua dat ngon tay */
    r.finger = true;

    static float ac_ir[WIN_LEN], ac_red[WIN_LEN];
    make_ac(ir, ac_ir);
    make_ac(red, ac_red);

    float s_ir = 0, s_red = 0, max_ir = 0;
    int cnt = 0;
    for (int i = 24; i < WIN_LEN; i++) {
        s_ir  += ac_ir[i]  * ac_ir[i];
        s_red += ac_red[i] * ac_red[i];
        if (ac_ir[i] > max_ir) max_ir = ac_ir[i];
        cnt++;
    }
    float rms_ir = sqrtf(s_ir / cnt), rms_red = sqrtf(s_red / cnt);

    /* Bien do AC qua nho => khong co tin hieu mach */
    if (rms_ir / dc_ir < 0.0003f) return r;

    /* ---- HR: dem khoang cach giua cac dinh ---- */
    float thr = 0.35f * max_ir;
    int last = -1, nint = 0, sum = 0;
    for (int i = 25; i < WIN_LEN - 1; i++) {
        if (ac_ir[i] > thr && ac_ir[i] > ac_ir[i - 1] && ac_ir[i] >= ac_ir[i + 1]) {
            if (last >= 0 && (i - last) < MIN_PEAK_DIST) continue;
            if (last >= 0) { sum += (i - last); nint++; }
            last = i;
        }
    }
    if (nint >= 2) {
        float hr = 60.0f * FS_HZ / ((float)sum / nint);
        if (hr >= 40 && hr <= 180) { r.hr = hr; r.hr_ok = true; }
    }

    /* ---- SpO2: R = (ACred/DCred) / (ACir/DCir) ---- */
    float R = (rms_red / dc_red) / (rms_ir / dc_ir);
    float spo2 = 110.0f - 25.0f * R;           /* cong thuc xap xi, can hieu chuan */
    if (R > 0.2f && R < 1.6f && spo2 > 70.0f) {
        r.spo2 = spo2 > 100.0f ? 100.0f : spo2;
        r.spo2_ok = true;
    }
    return r;
}

/* =================== I2C / main =================== */
static void i2c_setup(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t max_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &max_cfg, &s_max));

    /* Quet toan bo dia chi I2C, lay thiet bi dau tien khac MAX30102 lam LCD */
    uint8_t lcd_addr = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(s_bus, a, 50) == ESP_OK) {
            ESP_LOGI(TAG, "Thay thiet bi I2C o 0x%02X", a);
            if (a != MAX_ADDR && !lcd_addr) lcd_addr = a;
        }
    }

    if (lcd_addr) {
        i2c_device_config_t lcd_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = lcd_addr,
            .scl_speed_hz = 100000,            /* PCF8574 chay an toan o 100 kHz */
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &lcd_cfg, &s_lcd));
        ESP_LOGI(TAG, "LCD o dia chi 0x%02X", lcd_addr);
    } else {
        ESP_LOGW(TAG, "Khong thay LCD (0x27/0x3F), chi in ra Serial");
    }
}

void app_main(void)
{
    i2c_setup();
    lcd_init();
    lcd_row(0, "HR + SpO2 STEM");
    lcd_row(1, "Chi hoc tap");
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (max_init() != ESP_OK) {
        lcd_row(0, "Loi MAX30102!");
        lcd_row(1, "Kiem tra day");
        return;
    }
    lcd_row(0, "Dat ngon tay vao");
    lcd_row(1, "cam bien...");

    static float ir_w[WIN_LEN], red_w[WIN_LEN];
    int filled = 0, newcnt = 0;
    float hr_avg = 0, spo2_avg = 0;
    bool have = false;
    char l0[24], l1[24];

    while (1) {
        uint32_t r[32], ir[32];
        int n = max_read_samples(r, ir, 32);
        for (int k = 0; k < n; k++) {
            if (filled < WIN_LEN) {
                ir_w[filled] = (float)ir[k];
                red_w[filled] = (float)r[k];
                filled++;
            } else {
                memmove(ir_w, ir_w + 1, (WIN_LEN - 1) * sizeof(float));
                memmove(red_w, red_w + 1, (WIN_LEN - 1) * sizeof(float));
                ir_w[WIN_LEN - 1] = (float)ir[k];
                red_w[WIN_LEN - 1] = (float)r[k];
            }
            newcnt++;
        }

        if (filled >= WIN_LEN && newcnt >= STEP) {
            newcnt = 0;
            result_t res = analyze(ir_w, red_w);

            if (!res.finger) {
                have = false; hr_avg = 0; spo2_avg = 0;
                lcd_row(0, "Dat ngon tay vao");
                lcd_row(1, "cam bien...");
                ESP_LOGI(TAG, "Chua co ngon tay (IR=%.0f)", ir_w[WIN_LEN - 1]);
            } else {
                if (res.hr_ok)   hr_avg   = have && hr_avg   > 0 ? 0.7f * hr_avg   + 0.3f * res.hr   : res.hr;
                if (res.spo2_ok) spo2_avg = have && spo2_avg > 0 ? 0.7f * spo2_avg + 0.3f * res.spo2 : res.spo2;
                if (res.hr_ok || res.spo2_ok) have = true;

                if (hr_avg > 0)   snprintf(l0, sizeof l0, "HR:%.1f bpm", hr_avg);
                else              snprintf(l0, sizeof l0, "HR:-- bpm");
                if (spo2_avg > 0) snprintf(l1, sizeof l1, "SpO2:%.1f %%", spo2_avg);
                else              snprintf(l1, sizeof l1, "SpO2:--");

                lcd_row(0, l0);
                lcd_row(1, l1);
                ESP_LOGI(TAG, "HR=%.1f SpO2=%.1f | IR=%.0f RED=%.0f",
                         hr_avg, spo2_avg, ir_w[WIN_LEN - 1], red_w[WIN_LEN - 1]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
