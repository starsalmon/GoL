#include "board_config.h"

#if defined(BOARD_FNK0104)

#include "wifi_ota.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "include/secrets.h is missing. Copy include/secrets.example.h and fill it in."
#endif

#ifndef WIFI_SSID
#error "WIFI_SSID is not defined in include/secrets.h"
#endif
#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD is not defined in include/secrets.h"
#endif
#ifndef OTA_PASSWORD
#error "OTA_PASSWORD is not defined in include/secrets.h"
#endif

static bool started_ = false;
static bool otaBegun_ = false;
static bool loggedIp_ = false;
static bool loggedWait_ = false;
static uint32_t startMs_ = 0;
static char ipBuf_[16] = "";

void wifiOtaBegin() {
    WiFi.persistent(false);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);  // modem sleep; this board is not power-starved like the C3
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    started_ = true;
    startMs_ = millis();
    Serial.printf("Wi-Fi joining as %s (OTA port %u)\n", OTA_HOSTNAME, (unsigned)OTA_PORT);
}

static void startOta() {
    if (otaBegun_) return;
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        Serial.println("OTA start");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        if (total) {
            Serial.printf("OTA %u%%\r", (progress * 100u) / total);
        }
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA done, rebooting"); });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA error %u\n", (unsigned)error);
    });
    ArduinoOTA.begin();
    otaBegun_ = true;
}

void wifiOtaPoll() {
    if (!started_) return;

    if (WiFi.status() == WL_CONNECTED) {
        if (!loggedIp_) {
            const IPAddress ip = WiFi.localIP();
            snprintf(ipBuf_, sizeof(ipBuf_), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
            Serial.printf("Wi-Fi IP %s  OTA %s.local:%u\n",
                          ipBuf_, OTA_HOSTNAME, (unsigned)OTA_PORT);
            loggedIp_ = true;
        }
        startOta();
        ArduinoOTA.handle();
        return;
    }

    if (!loggedWait_ && (millis() - startMs_ > 15000)) {
        Serial.println("Wi-Fi not joined yet; will keep trying. Animation continues.");
        loggedWait_ = true;
    }
}

bool wifiOtaConnected() { return WiFi.status() == WL_CONNECTED; }

const char* wifiOtaIp() { return ipBuf_; }

#endif  // BOARD_FNK0104
