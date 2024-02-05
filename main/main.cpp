#include <chrono>
#include <thread>

#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "led_strip.hpp"
#include "logger.hpp"
#include "st7789.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// APA102 LED
static constexpr auto LED_STRIP_SPI_HOST = SPI3_HOST;
static constexpr auto LED_STRIP_MOSI_PIN = GPIO_NUM_40;
static constexpr auto LED_STRIP_SCLK_PIN = GPIO_NUM_39;
static constexpr auto LED_STRIP_CLK_SPEED_HZ = 10 * 1000 * 1000; // 10 MHz
static constexpr auto NUM_LEDS = 1;

// ST7735 TFT
static constexpr auto TFT_SPI_HOST = SPI2_HOST;
static constexpr auto TFT_MOSI_PIN = GPIO_NUM_3;
static constexpr auto TFT_SCLK_PIN = GPIO_NUM_5;
static constexpr auto TFT_CS_PIN = GPIO_NUM_4;
static constexpr auto TFT_DC_PIN = GPIO_NUM_2;
static constexpr auto TFT_RST_PIN = GPIO_NUM_1;
static constexpr auto TFT_CLK_SPEED_HZ = 40 * 1000 * 1000; // 40 MHz
static constexpr auto TFT_BACKLIGHT_PIN = GPIO_NUM_38;
static constexpr auto TFT_WIDTH = 160;
static constexpr auto TFT_HEIGHT = 80;

static constexpr auto tft_pixel_buffer_size = TFT_WIDTH * TFT_HEIGHT;
static constexpr bool tft_backlight_on_value = false;
static constexpr bool tft_reset_value = true;
static constexpr bool tft_invert_colors = false;
static constexpr auto tft_offset_x = 0;
static constexpr auto tft_offset_y = 0;
static constexpr bool tft_mirror_x = false;
static constexpr bool tft_mirror_y = false;
static constexpr auto tft_rotation = espp::Display::Rotation::LANDSCAPE;

using DisplayDriver = espp::St7789;

static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t);
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t);
extern "C" void IRAM_ATTR lcd_write(const uint8_t *data, size_t length, uint32_t user_data);
static void lcd_wait_lines();
extern "C" void IRAM_ATTR lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                        uint32_t user_data);
static spi_device_handle_t lcd_spi;
static const int lcd_spi_queue_size = 7;
static size_t lcd_num_queued_trans = 0;

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  espp::Logger logger({.tag = "T-Dongle S3", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  // make the SPI we'll use for the LED strip (a single APA102 RGB LED)
  spi_bus_config_t bus_config;
  memset(&bus_config, 0, sizeof(bus_config));
  bus_config.miso_io_num = -1;
  bus_config.mosi_io_num = LED_STRIP_MOSI_PIN;
  bus_config.sclk_io_num = LED_STRIP_SCLK_PIN;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz = 4 + 4 + NUM_LEDS * 4 ; // start frame + end frame + 4 bytes per LED
  spi_device_interface_config_t dev_config;
  memset(&dev_config, 0, sizeof(dev_config));
  dev_config.mode = 0;
  dev_config.clock_speed_hz = LED_STRIP_CLK_SPEED_HZ;
  dev_config.input_delay_ns = 0;
  dev_config.spics_io_num = -1;
  dev_config.queue_size = 1;
  spi_device_handle_t spi;
  spi_bus_initialize(LED_STRIP_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
  spi_bus_add_device(LED_STRIP_SPI_HOST, &dev_config, &spi);

  auto led_write = [&](const uint8_t *data, size_t len) {
    static spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi, &t);
  };

  // now make the LED strip
  espp::LedStrip led_strip(espp::LedStrip::Config{
      .num_leds = NUM_LEDS,
      .write = led_write,
      .send_brightness = true,
      .byte_order = espp::LedStrip::ByteOrder::BGR,
      .start_frame = {0x00, 0x00, 0x00, 0x00}, // APA102 start frame
      .end_frame = {0xFF, 0xFF, 0xFF, 0xFF},  // APA102 end frame
      .log_level = espp::Logger::Verbosity::INFO
    });
  led_strip.set_all(espp::Rgb(0, 0, 0));
  led_strip.show();

  // make a simple task that prints "Hello World!" every second
  espp::Task led_task({
      .name = "Led Task",
        .callback = [&](auto &m, auto &cv) -> bool {
          static auto start = std::chrono::high_resolution_clock::now();
          auto now = std::chrono::high_resolution_clock::now();
          float t = std::chrono::duration<float>(now - start).count();
          // rotate through rainbow colors in hsv based on time, hue is 0-360
          float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
          espp::Hsv hsv(hue, 1.0f, 1.0f);
          // full brightness (1.0, default) is _really_ bright, so tone it down
          led_strip.set_pixel(0, hsv, 0.05f);
          // show the new colors
          led_strip.show();
          std::unique_lock<std::mutex> lock(m);
          cv.wait_for(lock, 50ms);
          // we don't want to stop the task, so return false
          return false;
        },
        .stack_size_bytes = 4096,
        });
  led_task.start();

    // create the spi host
    spi_bus_config_t lcd_buscfg;
    memset(&lcd_buscfg, 0, sizeof(lcd_buscfg));
    lcd_buscfg.mosi_io_num = TFT_MOSI_PIN;
    lcd_buscfg.miso_io_num = -1;
    lcd_buscfg.sclk_io_num = TFT_SCLK_PIN;
    lcd_buscfg.quadwp_io_num = -1;
    lcd_buscfg.quadhd_io_num = -1;
    lcd_buscfg.max_transfer_sz = (int)(tft_pixel_buffer_size * sizeof(lv_color_t));
    // create the spi device
    spi_device_interface_config_t lcd_devcfg;
    memset(&lcd_devcfg, 0, sizeof(lcd_devcfg));
    lcd_devcfg.mode = 0;
    lcd_devcfg.clock_speed_hz = TFT_CLK_SPEED_HZ;
    lcd_devcfg.input_delay_ns = 0;
    lcd_devcfg.spics_io_num = TFT_CS_PIN;
    lcd_devcfg.queue_size = lcd_spi_queue_size;
    lcd_devcfg.pre_cb = lcd_spi_pre_transfer_callback;
    lcd_devcfg.post_cb = lcd_spi_post_transfer_callback;
    esp_err_t ret;
    // Initialize the SPI bus
    ret = spi_bus_initialize(TFT_SPI_HOST, &lcd_buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(TFT_SPI_HOST, &lcd_devcfg, &lcd_spi);
    ESP_ERROR_CHECK(ret);

    // initialize the controller
    DisplayDriver::initialize(espp::display_drivers::Config{
        .lcd_write = lcd_write,
        .lcd_send_lines = lcd_send_lines,
        .reset_pin = TFT_RST_PIN,
        .data_command_pin = TFT_DC_PIN,
        .reset_value = tft_reset_value,
        .invert_colors = tft_invert_colors,
        .offset_x = tft_offset_x,
        .offset_y = tft_offset_y,
        .mirror_x = tft_mirror_x,
        .mirror_y = tft_mirror_y,
    });
    // initialize the display / lvgl
    auto display = std::make_shared<espp::Display>(
        espp::Display::AllocatingConfig{.width = TFT_WIDTH,
                                        .height = TFT_HEIGHT,
                                        .pixel_buffer_size = tft_pixel_buffer_size,
                                        .flush_callback = DisplayDriver::flush,
                                        .backlight_pin = TFT_BACKLIGHT_PIN,
                                        .backlight_on_value = tft_backlight_on_value,
                                        .rotation = tft_rotation,
                                        .software_rotation_enabled = true});



  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
  }
}


// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

//! [pre_transfer_callback example]
// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  uint32_t user_flags = (uint32_t)(t->user);
  bool dc_level = user_flags & DC_LEVEL_BIT;
  gpio_set_level((gpio_num_t)TFT_DC_PIN, dc_level);
}
//! [pre_transfer_callback example]

//! [post_transfer_callback example]
// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  uint16_t user_flags = (uint32_t)(t->user);
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    lv_disp_flush_ready(disp->driver);
  }
}
//! [post_transfer_callback example]

//! [polling_transmit example]
extern "C" void IRAM_ATTR lcd_write(const uint8_t *data, size_t length, uint32_t user_data) {
  if (length == 0) {
    return;
  }
  static spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = length * 8;
  t.tx_buffer = data;
  t.user = (void *)user_data;
  spi_device_polling_transmit(lcd_spi, &t);
}
//! [polling_transmit example]

//! [queued_transmit example]
static void lcd_wait_lines() {
  spi_transaction_t *rtrans;
  esp_err_t ret;
  // Wait for all transactions to be done and get back the results.
  while (lcd_num_queued_trans) {
    // fmt::print("Waiting for {} lines\n", lcd_num_queued_trans);
    ret = spi_device_get_trans_result(lcd_spi, &rtrans, portMAX_DELAY);
    if (ret != ESP_OK) {
      fmt::print("Could not get trans result: {} '{}'\n", ret, esp_err_to_name(ret));
    }
    lcd_num_queued_trans--;
    // We could inspect rtrans now if we received any info back. The LCD is treated as write-only,
    // though.
  }
}

void IRAM_ATTR lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                              uint32_t user_data) {
  // if we haven't waited by now, wait here...
  lcd_wait_lines();
  esp_err_t ret;
  // Transaction descriptors. Declared static so they're not allocated on the stack; we need this
  // memory even when this function is finished because the SPI driver needs access to it even while
  // we're already calculating the next line.
  static spi_transaction_t trans[6];
  // In theory, it's better to initialize trans and data only once and hang on to the initialized
  // variables. We allocate them on the stack, so we need to re-init them each call.
  for (int i = 0; i < 6; i++) {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    if ((i & 1) == 0) {
      // Even transfers are commands
      trans[i].length = 8;
      trans[i].user = (void *)0;
    } else {
      // Odd transfers are data
      trans[i].length = 8 * 4;
      trans[i].user = (void *)DC_LEVEL_BIT;
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA;
  }
  size_t length = (xe - xs + 1) * (ye - ys + 1) * 2;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[0].tx_data[0] = (uint8_t)espp::Ili9341::Command::caset;
#endif
#if CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[0].tx_data[0] = (uint8_t)DisplayDriver::Command::caset;
#endif
  trans[1].tx_data[0] = (xs) >> 8;
  trans[1].tx_data[1] = (xs)&0xff;
  trans[1].tx_data[2] = (xe) >> 8;
  trans[1].tx_data[3] = (xe)&0xff;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[2].tx_data[0] = (uint8_t)espp::Ili9341::Command::raset;
#endif
#if CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[2].tx_data[0] = (uint8_t)DisplayDriver::Command::raset;
#endif
  trans[3].tx_data[0] = (ys) >> 8;
  trans[3].tx_data[1] = (ys)&0xff;
  trans[3].tx_data[2] = (ye) >> 8;
  trans[3].tx_data[3] = (ye)&0xff;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[4].tx_data[0] = (uint8_t)espp::Ili9341::Command::ramwr;
#endif
#if CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[4].tx_data[0] = (uint8_t)DisplayDriver::Command::ramwr;
#endif
  trans[5].tx_buffer = data;
  trans[5].length = length * 8;
  // undo SPI_TRANS_USE_TXDATA flag
  trans[5].flags = 0;
  // we need to keep the dc bit set, but also add our flags
  trans[5].user = (void *)(DC_LEVEL_BIT | user_data);
  // Queue all transactions.
  for (int i = 0; i < 6; i++) {
    ret = spi_device_queue_trans(lcd_spi, &trans[i], portMAX_DELAY);
    if (ret != ESP_OK) {
      fmt::print("Couldn't queue trans: {} '{}'\n", ret, esp_err_to_name(ret));
    } else {
      lcd_num_queued_trans++;
    }
  }
  // When we are here, the SPI driver is busy (in the background) getting the
  // transactions sent. That happens mostly using DMA, so the CPU doesn't have
  // much to do here. We're not going to wait for the transaction to finish
  // because we may as well spend the time calculating the next line. When that
  // is done, we can call send_line_finish, which will wait for the transfers
  // to be done and check their status.
}
//! [queued_transmit example]
