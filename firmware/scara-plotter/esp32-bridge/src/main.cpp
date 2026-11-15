/*
 * Bridge WiFi <-> seriale per il plotter SCARA (Arduino Mega + RAMPS + Marlin).
 *
 * Il programma principale sul PC si collega via TCP alla porta GCODE_PORT e invia G-code come
 * farebbe su una porta seriale; ogni byte passa in entrambe le direzioni senza essere interpretato,
 * quindi funzionano gli stessi programmi che parlano con Marlin via USB ("ok", "busy", "Error"...).
 *
 *   PC ──WiFi/TCP:8888──> ESP32 ──UART2 (115200)──> Mega Serial1 (SERIAL_PORT_2 di Marlin)
 *
 * Pagina di stato su http://scara-plotter.local/ , ultime risposte della macchina su /log.
 * Con STA non disponibile parte un punto di accesso (AP_SSID) a 192.168.4.1.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "config.h"

static WiFiServer gcodeServer(GCODE_PORT);
static WebServer web(80);
static WiFiClient client;
static bool apMode = false;

static uint32_t bytesToMega = 0, bytesFromMega = 0;
static uint32_t lastWifiCheck = 0;

// Ultime risposte della macchina (anello da 1 KB) per la pagina /log
static char logBuf[1024];
static size_t logHead = 0;
static bool logWrapped = false;

static void logByte(char c) {
  logBuf[logHead++] = c;
  if (logHead == sizeof(logBuf)) { logHead = 0; logWrapped = true; }
}

static String logText() {
  String s;
  if (logWrapped) s.concat(logBuf + logHead, sizeof(logBuf) - logHead);
  s.concat(logBuf, logHead);
  return s;
}

static void megaReset() {
  if (MEGA_RESET_PIN < 0) return;
  pinMode(MEGA_RESET_PIN, OUTPUT);
  digitalWrite(MEGA_RESET_PIN, LOW);
  delay(100);
  pinMode(MEGA_RESET_PIN, INPUT);   // rilascia il reset (alta impedenza)
}

static void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);             // senza risparmio energetico la latenza resta bassa
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(150);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi collegato: %s  IP %s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
    return;
  }
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("Rete non trovata: punto di accesso '%s' su %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

static String statusText() {
  String s;
  s += "SCARA plotter bridge\n";
  s += String("rete:        ") + (apMode ? "punto di accesso " AP_SSID : WIFI_SSID) + "\n";
  s += String("indirizzo:   ") + (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString() + "\n";
  if (!apMode) s += String("segnale:     ") + WiFi.RSSI() + " dBm\n";
  s += String("porta G-code: ") + GCODE_PORT + "\n";
  s += String("client:      ") + (client && client.connected() ? client.remoteIP().toString() : String("nessuno")) + "\n";
  s += String("byte verso la macchina:  ") + bytesToMega + "\n";
  s += String("byte dalla macchina:     ") + bytesFromMega + "\n";
  s += String("attivo da:   ") + millis() / 1000 + " s\n";
  return s;
}

static void setupWeb() {
  web.on("/", []() { web.send(200, "text/plain; charset=utf-8", statusText()); });
  web.on("/log", []() { web.send(200, "text/plain; charset=utf-8", logText()); });
  web.on("/reset", HTTP_POST, []() {
    if (MEGA_RESET_PIN < 0) { web.send(404, "text/plain", "reset non collegato"); return; }
    megaReset();
    web.send(200, "text/plain", "reset inviato");
  });
  web.begin();
}

static void setupOta() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

static void acceptClient() {
  WiFiClient c = gcodeServer.available();
  if (!c) return;
  if (client) client.stop();        // un solo client alla volta: il nuovo prende il posto del vecchio
  client = c;
  client.setNoDelay(true);          // niente ritardo di Nagle: ogni riga parte subito
  Serial.printf("client %s collegato\n", client.remoteIP().toString().c_str());
}

static void pump() {
  static uint8_t buf[128];

  // PC -> macchina, senza bloccare se la UART non riesce a stare dietro
  if (client && client.connected()) {
    int room = Serial2.availableForWrite();
    while (room > 0 && client.available() > 0) {
      int want = min(min(room, (int)sizeof(buf)), client.available());
      int n = client.read(buf, want);
      if (n <= 0) break;
      Serial2.write(buf, n);
      room -= n;
      bytesToMega += n;
    }
  }

  // macchina -> PC (se non c'è nessuno, le risposte restano solo nel log)
  int avail = Serial2.available();
  while (avail > 0) {
    int n = Serial2.readBytes(buf, min(avail, (int)sizeof(buf)));
    if (n <= 0) break;
    for (int i = 0; i < n; i++) logByte((char)buf[i]);
    bytesFromMega += n;
    if (client && client.connected()) client.write(buf, n);
    avail -= n;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  startWifi();
  if (MDNS.begin(HOSTNAME)) MDNS.addService("gcode", "tcp", GCODE_PORT);
  gcodeServer.begin();
  gcodeServer.setNoDelay(true);
  setupWeb();
  setupOta();
}

void loop() {
  static bool wasConnected = false;

  ArduinoOTA.handle();
  web.handleClient();
  acceptClient();
  pump();

  const bool connected = client && client.connected();
  if (wasConnected && !connected) {
    Serial.println("client scollegato");
    if (STOP_ON_DISCONNECT) Serial2.print("M410\n");
    client.stop();
  }
  wasConnected = connected;
  digitalWrite(LED_PIN, connected ? HIGH : ((millis() / 500) & 1));

  // la rete è sparita? riprova a collegarti ogni 10 s
  if (!apMode && WiFi.status() != WL_CONNECTED && millis() - lastWifiCheck > 10000) {
    lastWifiCheck = millis();
    WiFi.reconnect();
  }
}
