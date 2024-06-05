// code lifted from: https://github.com/moononournation/Arduino_GFX/#v1.4.7

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_pm.h"
#include "esp_private/gdma.h"
#include "hal/dma_types.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"

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
                uint16_t pclk_active_neg = 0, uint16_t de_idle_high = 0,
                uint16_t pclk_idle_high = 0)
      : de_(de), vsync_(vsync), hsync_(hsync), pclk_(pclk), r0_(r0), r1_(r1),
        r2_(r2), r3_(r3), r4_(r4), g0_(g0), g1_(g1), g2_(g2), g3_(g3), g4_(g4),
        g5_(g5), b0_(b0), b1_(b1), b2_(b2), b3_(b3), b4_(b4),
        hsync_polarity_(hsync_polarity), hsync_front_porch_(hsync_front_porch),
        hsync_pulse_width_(hsync_pulse_width),
        hsync_back_porch_(hsync_back_porch), vsync_polarity_(vsync_polarity),
        vsync_front_porch_(vsync_front_porch),
        vsync_pulse_width_(vsync_pulse_width),
        vsync_back_porch_(vsync_back_porch), pclk_active_neg_(pclk_active_neg),
        de_idle_high_(de_idle_high), pclk_idle_high_(pclk_idle_high) {}

  uint16_t *frame_buffer(int16_t width, int16_t height) {

    esp_lcd_rgb_panel_config_t *panel_config_ =
        (esp_lcd_rgb_panel_config_t *)heap_caps_calloc(
            1, sizeof(esp_lcd_rgb_panel_config_t),
            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    panel_config_->clk_src = LCD_CLK_SRC_PLL160M;
    panel_config_->timings.pclk_hz = speed_;
    panel_config_->timings.h_res = width;
    panel_config_->timings.v_res = height;
    // The following parameters should refer to LCD spec
    panel_config_->timings.hsync_pulse_width = hsync_pulse_width_;
    panel_config_->timings.hsync_back_porch = hsync_back_porch_;
    panel_config_->timings.hsync_front_porch = hsync_front_porch_;
    panel_config_->timings.vsync_pulse_width = vsync_pulse_width_;
    panel_config_->timings.vsync_back_porch = vsync_back_porch_;
    panel_config_->timings.vsync_front_porch = vsync_front_porch_;
    panel_config_->timings.flags.hsync_idle_low =
        (hsync_polarity_ == 0) ? 1 : 0;
    panel_config_->timings.flags.vsync_idle_low =
        (vsync_polarity_ == 0) ? 1 : 0;
    panel_config_->timings.flags.de_idle_high = de_idle_high_;
    panel_config_->timings.flags.pclk_active_neg = pclk_active_neg_;
    panel_config_->timings.flags.pclk_idle_high = pclk_idle_high_;

    panel_config_->data_width = 16;
    panel_config_->sram_trans_align = 8;
    panel_config_->psram_trans_align = 64;
    panel_config_->hsync_gpio_num = hsync_;
    panel_config_->vsync_gpio_num = vsync_;
    panel_config_->de_gpio_num = de_;
    panel_config_->pclk_gpio_num = pclk_;

    panel_config_->data_gpio_nums[0] = b0_;
    panel_config_->data_gpio_nums[1] = b1_;
    panel_config_->data_gpio_nums[2] = b2_;
    panel_config_->data_gpio_nums[3] = b3_;
    panel_config_->data_gpio_nums[4] = b4_;
    panel_config_->data_gpio_nums[5] = g0_;
    panel_config_->data_gpio_nums[6] = g1_;
    panel_config_->data_gpio_nums[7] = g2_;
    panel_config_->data_gpio_nums[8] = g3_;
    panel_config_->data_gpio_nums[9] = g4_;
    panel_config_->data_gpio_nums[10] = g5_;
    panel_config_->data_gpio_nums[11] = r0_;
    panel_config_->data_gpio_nums[12] = r1_;
    panel_config_->data_gpio_nums[13] = r2_;
    panel_config_->data_gpio_nums[14] = r3_;
    panel_config_->data_gpio_nums[15] = r4_;

    panel_config_->disp_gpio_num = GPIO_NUM_NC;

    panel_config_->flags.disp_active_low = 0;
    panel_config_->flags.relax_on_idle = 0;
    panel_config_->flags.fb_in_psram = true;

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(panel_config_, &panel_handle_));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_));

    LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = vsync_polarity_;
    LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = hsync_polarity_;

    return (uint16_t *)__containerof(panel_handle_, esp_rgb_panel_t, base)->fb;
  }

private:
  int32_t speed_ = 12000000L;
  int8_t de_, vsync_, hsync_, pclk_;
  int8_t r0_, r1_, r2_, r3_, r4_;
  int8_t g0_, g1_, g2_, g3_, g4_, g5_;
  int8_t b0_, b1_, b2_, b3_, b4_;
  uint16_t hsync_polarity_;
  uint16_t hsync_front_porch_;
  uint16_t hsync_pulse_width_;
  uint16_t hsync_back_porch_;
  uint16_t vsync_polarity_;
  uint16_t vsync_front_porch_;
  uint16_t vsync_pulse_width_;
  uint16_t vsync_back_porch_;
  uint16_t pclk_active_neg_;
  uint16_t de_idle_high_;
  uint16_t pclk_idle_high_;

  esp_lcd_panel_handle_t panel_handle_ = nullptr;
};
