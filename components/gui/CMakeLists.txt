idf_component_register(
  SRC_DIRS "src" "generated" "generated/screens" "generated/components"
  INCLUDE_DIRS "include"
  PRIV_INCLUDE_DIRS "generated"
  REQUIRES lvgl task display logger)
add_definitions(-DLV_COLOR_16_SWAP=1)
