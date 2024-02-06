#include "gui.hpp"

#include "ui.h"

void Gui::init_ui() {
  ui_init();
}

void Gui::set_label(const std::string_view &label) {
  std::scoped_lock<std::recursive_mutex> lk(mutex_);
  lv_label_set_text(ui_Label2, label.data());
}

void Gui::set_meter(size_t value, bool animate) {
  std::scoped_lock<std::recursive_mutex> lk(mutex_);
  if (animate) {
    lv_bar_set_value(ui_Bar1, value, LV_ANIM_ON);
  } else {
    lv_bar_set_value(ui_Bar1, value, LV_ANIM_OFF);
  }
}
