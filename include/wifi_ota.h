#pragma once

#if defined(BOARD_FNK0104)

#include <stdint.h>

// Wi-Fi station + ArduinoOTA for the Freenove board. Credentials come
// from include/secrets.h (gitignored). Failures are not fatal: the
// animation keeps running if the AP is out of reach.
void wifiOtaBegin();
void wifiOtaPoll();
bool wifiOtaConnected();
const char* wifiOtaIp();  // empty string until joined

#endif
