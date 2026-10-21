/*
 * Seeed XIAO ESP32-C3 <-> EmStat Pico SWV controller with BLE (Nordic UART service).
 *
 * Wiring:  XIAO D6 (GPIO21, TX) -> Pico RX
 *          XIAO D7 (GPIO20, RX) <- Pico TX
 *          GND shared
 *          D10 (GPIO10)         -> function generator enable (HIGH = on)
 *
 * Arduino IDE: board "XIAO_ESP32C3", esp32 core 2.x or 3.x,
 *              Tools > USB CDC On Boot > Enabled (so Serial works over USB).
 *
 * Control : send the text START over BLE (the web page does this) or USB serial.
 * Output  : USB serial -> raw MethodSCRIPT + '#' status lines (works with swv_monitor.py)
 *           BLE        -> '#' status lines + decoded CSV rows: potential_mV,net_uA,fwd_uA,rev_uA
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- User settings ----------------
const uint8_t  GEN_PIN              = 10;
const uint32_t GEN_ON_MS            = 5UL * 60UL * 1000UL;   // 5 minutes
const bool     KEEP_GEN_ON_DURING_SWV = false;
const uint32_t PICO_BAUD            = 230400;
const int      PICO_RX_PIN          = 20;   // D7
const int      PICO_TX_PIN          = 21;   // D6
const uint32_t PICO_IDLE_TIMEOUT_MS = 30000;

#define E_BEGIN  "-500m"   // ASSUMED potential window - edit for your cell
#define E_END    "500m"
#define E_STEP   "10m"
#define E_AMP    "100m"
#define FREQ_HZ  "17"

const char* const SCRIPT[] = {
  "e",
  "var p", "var c", "var f", "var r",
  "set_pgstat_chan 0",
  "set_pgstat_mode 3",
  "set_max_bandwidth 200",
  "set_range_minmax da " E_BEGIN " " E_END,
  "set_range ba 100u",
  "set_autoranging ba 100n 100u",
  "set_e " E_BEGIN,
  "cell_on",
  "wait 1",
  "meas_loop_swv p c f r " E_BEGIN " " E_END " " E_STEP " " E_AMP " " FREQ_HZ,
  "pck_start", "pck_add p", "pck_add c", "pck_add f", "pck_add r", "pck_end",
  "endloop",
  "on_finished:",
  "cell_off",
  ""
};
const size_t SCRIPT_LINES = sizeof(SCRIPT) / sizeof(SCRIPT[0]);

// ---------------- BLE (Nordic UART service) ----------------
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_CHAR "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // phone -> device (write)
#define NUS_TX_CHAR "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // device -> phone (notify)
const size_t BLE_CHUNK = 20;                                  // safe for default MTU

BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;
volatile bool startFlag = false;
String cmdBuf;

void feedCommandChar(char ch) {
  cmdBuf += ch;
  if (cmdBuf.length() > 16) cmdBuf.remove(0, cmdBuf.length() - 16);
  String low = cmdBuf;
  low.toLowerCase();
  if (low.endsWith("start")) { cmdBuf = ""; startFlag = true; }   // newline not required
}

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) { bleConnected = true; }
  void onDisconnect(BLEServer*) { bleConnected = false; BLEDevice::startAdvertising(); }
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    auto v = c->getValue();                 // std::string (core 2.x) or String (3.x)
    for (size_t i = 0; i < v.length(); i++) feedCommandChar((char)v[i]);
  }
};

void bleSend(const String& s) {
  if (!bleConnected || !txChar) return;
  for (size_t i = 0; i < s.length(); i += BLE_CHUNK) {
    String part = s.substring(i, i + BLE_CHUNK);
    txChar->setValue((uint8_t*)part.c_str(), part.length());
    txChar->notify();
    delay(4);                               // avoid flooding the BLE stack
  }
}

void setupBle() {
  BLEDevice::init("SWV-Seeed");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService* svc = server->createService(NUS_SERVICE);

  txChar = svc->createCharacteristic(NUS_TX_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic* rxChar = svc->createCharacteristic(
      NUS_RX_CHAR, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCb());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ---------------- Output helpers ----------------
void status(const String& msg) {
  Serial.print("# ");
  Serial.println(msg);
  bleSend("# " + msg + "\n");
}

double parseVar(String t) {
  int comma = t.indexOf(',');
  if (comma >= 0) t = t.substring(0, comma);
  if (t.length() < 10) return NAN;
  long raw = strtol(t.substring(2, 9).c_str(), NULL, 16) - (1L << 27);
  double f = 1.0;
  switch (t[9]) {
    case 'a': f = 1e-18; break;  case 'f': f = 1e-15; break;
    case 'p': f = 1e-12; break;  case 'n': f = 1e-9;  break;
    case 'u': f = 1e-6;  break;  case 'm': f = 1e-3;  break;
    case 'k': f = 1e3;   break;  case 'M': f = 1e6;   break;
    case 'G': f = 1e9;   break;  default:  f = 1.0;   break;
  }
  return raw * f;
}

void sendDecodedToBle(const String& line) {
  if (!bleConnected) return;
  double v[4];
  int n = 0, start = 1;
  while (n < 4 && start < (int)line.length()) {
    int end = line.indexOf(';', start);
    if (end < 0) end = line.length();
    v[n++] = parseVar(line.substring(start, end));
    start = end + 1;
  }
  if (n < 4) return;
  String row = String(v[0] * 1e3, 2) + "," + String(v[1] * 1e6, 4) + "," +
               String(v[2] * 1e6, 4) + "," + String(v[3] * 1e6, 4) + "\n";
  bleSend(row);
}

// ---------------- Stages ----------------
bool pollUsbForStart() {
  while (Serial.available()) feedCommandChar((char)Serial.read());
  if (startFlag) { startFlag = false; return true; }
  return false;
}

void runGenerator() {
  digitalWrite(GEN_PIN, HIGH);
  status("GENERATOR ON (5 min)");
  uint32_t t0 = millis(), lastReport = 0;
  while (millis() - t0 < GEN_ON_MS) {
    if (millis() - lastReport >= 5000) {
      lastReport = millis();
      status("generator remaining s: " + String((GEN_ON_MS - (millis() - t0)) / 1000));
    }
    delay(50);
  }
  if (!KEEP_GEN_ON_DURING_SWV) { digitalWrite(GEN_PIN, LOW); status("GENERATOR OFF"); }
}

void sendScriptToPico() {
  while (Serial1.available()) Serial1.read();
  for (size_t i = 0; i < SCRIPT_LINES; i++) {
    Serial1.print(SCRIPT[i]);
    Serial1.print('\n');
    Serial1.flush();
    delay(5);
  }
  status("SCRIPT SENT");
}

void relayPicoOutput() {
  uint32_t lastRx = millis();
  String line;
  bool sawLoopEnd = false;
  while (millis() - lastRx < PICO_IDLE_TIMEOUT_MS) {
    while (Serial1.available()) {
      char ch = Serial1.read();
      lastRx = millis();
      Serial.write(ch);                      // raw to USB
      if (ch == '\n') {
        if (line.length() && line[0] == 'P') sendDecodedToBle(line);
        if (line == "*" || line == "*\r") sawLoopEnd = true;
        else if (sawLoopEnd && (line.length() == 0 || line == "\r")) return;
        line = "";
      } else if (line.length() < 200) {
        line += ch;
      }
    }
    delay(1);
  }
  status("TIMEOUT waiting for Pico");
}

// ---------------- Arduino entry points ----------------
void setup() {
  pinMode(GEN_PIN, OUTPUT);
  digitalWrite(GEN_PIN, LOW);

  Serial.begin(115200);
  delay(500);
  Serial1.begin(PICO_BAUD, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

  setupBle();
}

void loop() {
  status("READY - send START");
  uint32_t lastPing = millis();
  while (!(pollUsbForStart() || startFlag)) {
    if (millis() - lastPing > 3000) { status("READY - send START"); lastPing = millis(); }
    delay(10);
  }
  startFlag = false;

  status("START received");
  runGenerator();
  status("STARTING SWV");
  sendScriptToPico();
  relayPicoOutput();
  digitalWrite(GEN_PIN, LOW);
  status("DONE");
}
