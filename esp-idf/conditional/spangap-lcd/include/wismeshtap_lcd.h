/**
 * wismeshtap_lcd.h — the WisMesh TAP V2's on-device input HAL, as a Service.
 *
 * WismeshTapLcdInput registers the FT5x06 touch + Home-button input HAL with
 * spangap-lcd in onStart (start band, before lcdInit()). when:-gated on
 * spangap-lcd via the straddle.yaml `services:` entry, so the whole slice (this
 * header + wismeshtap_lcd.cpp) compiles only in an LCD build. Declared here
 * (global) for the generated trampoline; defined in wismeshtap_lcd.cpp.
 */
#pragma once

#include "service.h"

class WismeshTapLcdInput : public Service {
public:
    void onStart() override;   /* input HAL register */
};
