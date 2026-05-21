// =============================================================
//  SECRETS (plantilla) — copia este archivo a secrets.h
//  y completa los valores. secrets.h NO se sube a Git.
// =============================================================
#ifndef SECRETS_H
#define SECRETS_H

#define WIFI_SSID       "TU_SSID"
#define WIFI_PASSWORD   "TU_PASSWORD"

// Backend.
//
// LOCAL (desarrollo en LAN):
//   #define SERVER_BASE  "http://192.168.1.53:8001"
//
// REMOTO (despliegue en Render.com, con HTTPS):
//   #define SERVER_BASE  "https://germinador-iot.onrender.com"
//
// El firmware detecta automáticamente si la URL es HTTPS y usa
// WiFiClientSecure si hace falta.
#define SERVER_BASE     "https://TU-PROYECTO.onrender.com"

#define DEVICE_TOKEN    "token-largo-y-secreto-compartido-con-el-backend"

#endif
