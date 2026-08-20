# hw-wismesh-tap-v2 board HAL — an ordinary spangap component (not a buildable).
#
# Compiles the RAK WisMesh TAP V2 board code: power rails/CS park/battery
# (wismeshtap.cpp). The Home-button input HAL lives in conditional/spangap-lcd/
# and is compiled ONLY when spangap-lcd is staged. The FT5x06 touch is
# spangap-lcd's own CONFIG_LCD_TOUCH_* controller and the GNSS receiver is the
# generic spangap/gps straddle — this board only supplies their pins.
#
# app_main is NOT here — it's generated into the reticulous/reticulous buildable
# that adds this board with `--with`. This component just exports the board's
# Service classes that the generated dispatcher constructs and walks.
include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
idf_component_register(
    SRCS
        "src/wismeshtap.cpp"
        # This board's one self-assertion (detect_hw), called by
        # spangapInit() before any bus is claimed.
        "src/detect.cpp"
        # conditional/<name>/ board code, compiled only when <name> is staged
        # (conditional/spangap-lcd/src/wismeshtap_lcd.cpp).
        ${SPANGAP_CONDITIONAL_SRCS}
    INCLUDE_DIRS "include" ${SPANGAP_CONDITIONAL_INCLUDE_DIRS}
    REQUIRES
        ${SPANGAP_REQUIRES}
        driver
        esp_adc          # battery voltage ADC on GPIO 1
        esp_timer
        esp_event
        esp_netif
        esp_wifi
        lwip
)
