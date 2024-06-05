// code lifted from: https://github.com/moononournation/Arduino_GFX/#v1.4.7

#include "esp32s3/rom/cache.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_pm.h"
#include "esp_private/gdma.h"
#include "hal/dma_types.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"

#define GFX_NOT_DEFINED -1
#define GPIO_NUM_NC -1

// This function is located in ROM (also see
// esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

// extract from esp-idf esp_lcd_rgb_panel.c
struct esp_rgb_panel_t {
  esp_lcd_panel_t base;  // Base class of generic lcd panel
  int panel_id;          // LCD panel ID
  lcd_hal_context_t hal; // Hal layer object
  size_t data_width; // Number of data lines (e.g. for RGB565, the data width is
                     // 16)
  size_t sram_trans_align;  // Alignment for framebuffer that allocated in SRAM
  size_t psram_trans_align; // Alignment for framebuffer that allocated in PSRAM
  int disp_gpio_num;  // Display control GPIO, which is used to perform action
                      // like "disp_off"
  intr_handle_t intr; // LCD peripheral interrupt handle
  esp_pm_lock_handle_t pm_lock; // Power management lock
  size_t num_dma_nodes; // Number of DMA descriptors that used to carry the
                        // frame buffer
  uint8_t *fb;          // Frame buffer
  size_t fb_size;       // Size of frame buffer
  int data_gpio_nums[SOC_LCD_RGB_DATA_WIDTH]; // GPIOs used for data lines, we
                                              // keep these GPIOs for action
                                              // like "invert_color"
  size_t resolution_hz;                       // Peripheral clock resolution
  esp_lcd_rgb_timing_t
      timings; // RGB timing parameters (e.g. pclk, sync pulse, porch width)
  gdma_channel_handle_t dma_chan; // DMA channel handle
  esp_lcd_rgb_panel_frame_trans_done_cb_t
      on_frame_trans_done; // Callback, invoked after frame trans done
  void *user_ctx;          // Reserved user's data of callback functions
  int x_gap; // Extra gap in x coordinate, it's used when calculate the flush
             // window
  int y_gap; // Extra gap in y coordinate, it's used when calculate the flush
             // window
  struct {
    unsigned int disp_en_level : 1; // The level which can turn on the screen by
                                    // `disp_gpio_num`
    unsigned int stream_mode : 1;   // If set, the LCD transfers data
                                    // continuously, otherwise, it stops
                                    // refreshing the LCD when transaction done
    unsigned int fb_in_psram : 1;   // Whether the frame buffer is in PSRAM
  } flags;
  dma_descriptor_t dma_nodes[]; // DMA descriptor pool of size `num_dma_nodes`
};

class ESP32RGBPanel {
public:
  ESP32RGBPanel(int8_t de, int8_t vsync, int8_t hsync, int8_t pclk, int8_t r0,
                int8_t r1, int8_t r2, int8_t r3, int8_t r4, int8_t g0,
                int8_t g1, int8_t g2, int8_t g3, int8_t g4, int8_t g5,
                int8_t b0, int8_t b1, int8_t b2, int8_t b3, int8_t b4,
                uint16_t hsync_polarity, uint16_t hsync_front_porch,
                uint16_t hsync_pulse_width, uint16_t hsync_back_porch,
                uint16_t vsync_polarity, uint16_t vsync_front_porch,
                uint16_t vsync_pulse_width, uint16_t vsync_back_porch,
                uint16_t pclk_active_neg = 0,
                int32_t prefer_speed = GFX_NOT_DEFINED,
                uint16_t de_idle_high = 0, uint16_t pclk_idle_high = 0)
      : _de(de), _vsync(vsync), _hsync(hsync), _pclk(pclk), _r0(r0), _r1(r1),
        _r2(r2), _r3(r3), _r4(r4), _g0(g0), _g1(g1), _g2(g2), _g3(g3), _g4(g4),
        _g5(g5), _b0(b0), _b1(b1), _b2(b2), _b3(b3), _b4(b4),
        _hsync_polarity(hsync_polarity), _hsync_front_porch(hsync_front_porch),
        _hsync_pulse_width(hsync_pulse_width),
        _hsync_back_porch(hsync_back_porch), _vsync_polarity(vsync_polarity),
        _vsync_front_porch(vsync_front_porch),
        _vsync_pulse_width(vsync_pulse_width),
        _vsync_back_porch(vsync_back_porch), _pclk_active_neg(pclk_active_neg),
        _prefer_speed(prefer_speed), _de_idle_high(de_idle_high),
        _pclk_idle_high(pclk_idle_high) {}

  uint16_t *frame_buffer(int16_t w, int16_t h) {

    esp_lcd_rgb_panel_config_t *_panel_config =
        (esp_lcd_rgb_panel_config_t *)heap_caps_calloc(
            1, sizeof(esp_lcd_rgb_panel_config_t),
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    _panel_config->clk_src = LCD_CLK_SRC_PLL160M;
    _panel_config->timings.pclk_hz =
        (_prefer_speed == GFX_NOT_DEFINED) ? _speed : _prefer_speed;
    _panel_config->timings.h_res = w;
    _panel_config->timings.v_res = h;
    // The following parameters should refer to LCD spec
    _panel_config->timings.hsync_pulse_width = _hsync_pulse_width;
    _panel_config->timings.hsync_back_porch = _hsync_back_porch;
    _panel_config->timings.hsync_front_porch = _hsync_front_porch;
    _panel_config->timings.vsync_pulse_width = _vsync_pulse_width;
    _panel_config->timings.vsync_back_porch = _vsync_back_porch;
    _panel_config->timings.vsync_front_porch = _vsync_front_porch;
    _panel_config->timings.flags.hsync_idle_low =
        (_hsync_polarity == 0) ? 1 : 0;
    _panel_config->timings.flags.vsync_idle_low =
        (_vsync_polarity == 0) ? 1 : 0;
    _panel_config->timings.flags.de_idle_high = _de_idle_high;
    _panel_config->timings.flags.pclk_active_neg = _pclk_active_neg;
    _panel_config->timings.flags.pclk_idle_high = _pclk_idle_high;

    _panel_config->data_width =
        16; // RGB565 in parallel mode, thus 16bit in width
    _panel_config->sram_trans_align = 8;
    _panel_config->psram_trans_align = 64;
    _panel_config->hsync_gpio_num = _hsync;
    _panel_config->vsync_gpio_num = _vsync;
    _panel_config->de_gpio_num = _de;
    _panel_config->pclk_gpio_num = _pclk;

    _panel_config->data_gpio_nums[0] = _b0;
    _panel_config->data_gpio_nums[1] = _b1;
    _panel_config->data_gpio_nums[2] = _b2;
    _panel_config->data_gpio_nums[3] = _b3;
    _panel_config->data_gpio_nums[4] = _b4;
    _panel_config->data_gpio_nums[5] = _g0;
    _panel_config->data_gpio_nums[6] = _g1;
    _panel_config->data_gpio_nums[7] = _g2;
    _panel_config->data_gpio_nums[8] = _g3;
    _panel_config->data_gpio_nums[9] = _g4;
    _panel_config->data_gpio_nums[10] = _g5;
    _panel_config->data_gpio_nums[11] = _r0;
    _panel_config->data_gpio_nums[12] = _r1;
    _panel_config->data_gpio_nums[13] = _r2;
    _panel_config->data_gpio_nums[14] = _r3;
    _panel_config->data_gpio_nums[15] = _r4;

    _panel_config->disp_gpio_num = GPIO_NUM_NC;

    _panel_config->flags.disp_active_low = 0;
    _panel_config->flags.relax_on_idle = 0;
    _panel_config->flags.fb_in_psram = 1; // allocate frame buffer in PSRAM

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(_panel_config, &_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(_panel_handle));

    _rgb_panel = __containerof(_panel_handle, esp_rgb_panel_t, base);

    LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = _vsync_polarity;
    LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = _hsync_polarity;

    return (uint16_t *)_rgb_panel->fb;
  }

private:
  int32_t _speed = 12000000L;
  int8_t _de, _vsync, _hsync, _pclk;
  int8_t _r0, _r1, _r2, _r3, _r4;
  int8_t _g0, _g1, _g2, _g3, _g4, _g5;
  int8_t _b0, _b1, _b2, _b3, _b4;
  uint16_t _hsync_polarity;
  uint16_t _hsync_front_porch;
  uint16_t _hsync_pulse_width;
  uint16_t _hsync_back_porch;
  uint16_t _vsync_polarity;
  uint16_t _vsync_front_porch;
  uint16_t _vsync_pulse_width;
  uint16_t _vsync_back_porch;
  uint16_t _pclk_active_neg;
  int32_t _prefer_speed = 12000000L;
  uint16_t _de_idle_high;
  uint16_t _pclk_idle_high;

  esp_lcd_panel_handle_t _panel_handle = NULL;
  esp_rgb_panel_t *_rgb_panel;
};
