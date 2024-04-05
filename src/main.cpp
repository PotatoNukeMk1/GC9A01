#include <Arduino.h>
#include <SPIFFS.h>
// #include <FFat.h>
#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>
#include <driver/spi_master.h>
#include <lvgl.h>

#define LCD_MISO  MISO
#define LCD_MOSI  MOSI
#define LCD_CLK   SCK
#define LCD_CS    A0
#define LCD_DC    A1
#define LCD_RST   A2
#define LCD_BL    A3
#define LCD_H_RES 240
#define LCD_V_RES 240
#define DISP_BUF_SIZE (LCD_H_RES * (LCD_V_RES / 10) * (LV_COLOR_DEPTH / 8))

const char *TAG = "GC9A01 Demo";

bool disp_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
  lv_display_flush_ready(static_cast<lv_display_t *>(user_ctx));
  return false;
}

void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
  lv_draw_sw_rgb565_swap(px_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
  esp_lcd_panel_handle_t panel_handle = static_cast<esp_lcd_panel_handle_t>(lv_display_get_user_data(disp));
  esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
}

void set_angle(void * obj, int32_t v)
{
  lv_arc_set_value((lv_obj_t *)obj, v);
}

void setup() {
  Serial.begin(115200);
  // while(Serial);

  // if(!FFat.begin()) {
  //   Serial.println(F("Error: failed to mount FFat"));
  //   while(true) {
  //     delay(10);
  //   }
  // }

  if(!SPIFFS.begin()) {
    Serial.println(F("Error: failed to mount SPIFFS"));
    while(true) {
      delay(10);
    }
  }

  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 255);

  lv_init();

  lv_display_t *disp;
  disp = lv_display_create(LCD_H_RES, LCD_V_RES);

  ESP_LOGI(TAG, "Initialize SPI bus");
  spi_bus_config_t buscfg = {
    .mosi_io_num = LCD_MOSI,
    .miso_io_num = LCD_MISO,
    .sclk_io_num = LCD_CLK,
    .max_transfer_sz = DISP_BUF_SIZE,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
    .cs_gpio_num = LCD_CS,
    .dc_gpio_num = LCD_DC,
    .spi_mode = 0,
    .pclk_hz = 20000000,
    .trans_queue_depth = 20,
    .on_color_trans_done = disp_flush_ready,
    .user_ctx = disp,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

  ESP_LOGI(TAG, "Install GC9A01 panel driver");
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = LCD_RST,
    .color_space = ESP_LCD_COLOR_SPACE_BGR,
    .bits_per_pixel = 16,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
  
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  void *buf_1 = heap_caps_malloc(DISP_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  void *buf_2 = heap_caps_malloc(DISP_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  // void *buf_1 = ps_malloc(DISP_BUF_SIZE);
  // void *buf_2 = ps_malloc(DISP_BUF_SIZE);

  lv_display_set_flush_cb(disp, disp_flush);
  lv_display_set_user_data(disp, panel_handle);
  lv_display_set_buffers(disp, buf_1, buf_2, DISP_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_obj_t * img1 = lv_image_create(lv_screen_active());
  lv_image_set_src(img1, "S:/wallpaper.png");
  lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t * arc = lv_arc_create(lv_screen_active());
  lv_obj_set_size(arc, 240, 240);
  lv_arc_set_rotation(arc, 270);
  lv_arc_set_bg_angles(arc, 0, 360);
  lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(arc);

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, arc);
  lv_anim_set_exec_cb(&a, set_angle);
  lv_anim_set_duration(&a, 1000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_repeat_delay(&a, 500);
  lv_anim_set_values(&a, 0, 100);
  lv_anim_start(&a);
  
  ESP_LOGI(TAG, "Ready");
}

void loop() {
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);
}
