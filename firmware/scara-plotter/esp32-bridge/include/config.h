#pragma once
/*
 * Configurazione del bridge. Le credenziali WiFi stanno in include/secrets.h
 * (non versionato): copia secrets.example.h in secrets.h e compilalo.
 */
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #include "secrets.example.h"
#endif

#define HOSTNAME        "scara-plotter"   // raggiungibile come scara-plotter.local
#define GCODE_PORT      8888              // porta TCP del G-code

// Collegamento al Mega (Serial1 di Marlin: SERIAL_PORT_2 1, pin 18 TX1 / 19 RX1)
#define MEGA_BAUD       115200            // deve coincidere con hardware.seriale.esp32_baud
#define MEGA_RX_PIN     16                // ESP32 RX2  <- TX1 del Mega (pin 18) tramite partitore 1k/2k
#define MEGA_TX_PIN     17                // ESP32 TX2  -> RX1 del Mega (pin 19)

// Reset del Mega (opzionale): GPIO collegato al pin RESET della RAMPS. -1 = disattivato
#define MEGA_RESET_PIN  -1

#define LED_PIN         2                 // LED blu della scheda: fisso = client TCP collegato

#define WIFI_TIMEOUT_MS 20000             // dopo questo tempo senza rete parte il punto di accesso
#define AP_SSID         "SCARA-PLOTTER"
#define AP_PASSWORD     "scara1234"       // minimo 8 caratteri

// Se il client TCP cade durante il disegno, ferma i movimenti in coda (M410)
#define STOP_ON_DISCONNECT 0
