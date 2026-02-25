// main/secrets.example.h
#pragma once

// Wi-Fi Config
#define ROUTER_SSID              "YOUR_WIFI_SSID"
#define ROUTER_PASS              "YOUR_WIFI_PASSWORD"

// ESP-NOW Security
#define ESPNOW_PMK               "0123456789abcdef"   // 16-byte PMK

// InfluxDB Cloud Config
#define INFLUXDB_URL             "https://your-region.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN           "YOUR_INFLUXDB_TOKEN"
#define INFLUXDB_ORG             "YOUR_INFLUXDB_ORG"
#define INFLUXDB_BUCKET          "YOUR_INFLUXDB_BUCKET"
#define INFLUXDB_MEASUREMENT     "mesh_sensor"

// Alert Configuration
#define ALERTS_INFLUXDB_TOKEN    "YOUR_ALERTS_INFLUXDB_TOKEN"
#define ALERTS_DB                "alerts"
