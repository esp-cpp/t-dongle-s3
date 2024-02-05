#include <chrono>
#include <thread>

#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "led_strip.hpp"
#include "logger.hpp"
#include "st7789.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// ST7735 TFT
static constexpr auto TFT_SPI_HOST = SPI2_HOST;
static constexpr auto TFT_MOSI_PIN = 3;
static constexpr auto TFT_MISO_PIN = -1;
static constexpr auto TFT_SCLK_PIN = 5;
static constexpr auto TFT_CS_PIN = 4;
static constexpr auto TFT_DC_PIN = 2;
static constexpr auto TFT_RST_PIN = 1;
static constexpr auto TFT_CLK_SPEED_HZ = 40 * 1000 * 1000; // 40 MHz
static constexpr auto TFT_BACKLIGHT_PIN = 38;
static constexpr auto TFT_WIDTH = 160;
static constexpr auto TFT_HEIGHT = 80;

// APA102 LED
static constexpr auto LED_STRIP_SPI_HOST = SPI3_HOST;
static constexpr auto LED_STRIP_MOSI_PIN = 40;
static constexpr auto LED_STRIP_SCLK_PIN = 39;
static constexpr auto LED_STRIP_CLK_SPEED_HZ = 10 * 1000 * 1000; // 10 MHz
static constexpr auto NUM_LEDS = 1;

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

  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
  }
}
