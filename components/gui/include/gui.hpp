#pragma once

#include <memory>
#include <mutex>

#include "display.hpp"
#include "logger.hpp"
#include "task.hpp"

class Gui {
public:
  struct Config {
    std::shared_ptr<espp::Display> display;
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit Gui(const Config &config)
      : display_(config.display), logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    // now start the gui updater task
    using namespace std::placeholders;
    task_ = espp::Task::make_unique({.name = "Gui Task",
                                     .callback = std::bind(&Gui::update, this, _1, _2),
                                     .stack_size_bytes = 6 * 1024});
    task_->start();
  }

  void set_label(const std::string_view &label);

  void set_meter(size_t value, bool animate = true);

protected:
  void init_ui();

  bool update(std::mutex &m, std::condition_variable &cv) {
    {
      std::scoped_lock<std::recursive_mutex> lk(mutex_);
      lv_task_handler();
    }
    {
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 16ms);
    }
    // don't want to stop the task
    return false;
  }

  std::shared_ptr<espp::Display> display_;
  std::unique_ptr<espp::Task> task_;
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
