#pragma once

// WiFi credentials
static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Google Apps Script deployment and Drive folders
// Web App URL format: https://script.google.com/macros/s/DEPLOYMENT_ID/exec
static const char* GAS_DEPLOYMENT_ID = "YOUR_DEPLOYMENT_ID";  // replace with the part between /s/ and /exec
static const char* DRIVE_MAIN_FOLDER = "ESP32-CAM";            // or any folder name you prefer
static const char* DEVICE_LABEL = "House_1";                   // subfolder per device/location
