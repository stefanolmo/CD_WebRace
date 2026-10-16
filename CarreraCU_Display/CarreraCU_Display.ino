/*
  Carrera Digital 124/132 Control Unit -> Freenove ESP32-S3 CYD 2.8" (FNK0104B)

  - UART verso la Control Unit (connettore P2) con stato della connessione.
  - Configurazione WiFi dal touchscreen (tastiera a schermo, scansione reti).
  - SSID e password salvati in NVS (Preferences): sopravvivono al riavvio e si
    possono modificare o cancellare quando vuoi.

  Librerie (Library Manager): LovyanGFX. WiFi e Preferences sono nel core ESP32.
  Board: ESP32S3 Dev Module, "USB CDC On Boot: Enabled".

  Pin verificati sul setup ufficiale Freenove FNK0104AB e sul suo sketch del touch:
    Display ILI9341: SCLK 12, MOSI 11, MISO 13, CS 10, DC 46, BL 45, RST non usato,
                     colori BGR + inversione attiva.
    Touch FT6336:    SDA 16, SCL 15, RST 18, INT 17.
    UART CU:         P2 = GPIO44 (RXD0) e GPIO43 (TXD0). L'USB e' nativa.
                     La ROM scrive i log di boot su TXD0 a 115200 baud: all'accensione
                     la CU puo' ricevere qualche byte spurio.

  !!! DA VERIFICARE PRIMA DI COLLEGARE LA CU !!!
  - Livelli logici: l'ESP32 e' a 3.3 V. Misurare la porta della CU; se e' a 5 V
    serve un level shifter. Pinout fisico della porta della CU: da verificare.
  - Protocollo (19200 8N1, frame '"' ... '$', checksum 1 char): ricostruito dal
    progetto open source "carreralib", non verificato su una CU reale. Se non si
    connette, la riga "RX" della home mostra i byte grezzi ricevuti.
  - La password WiFi e' salvata in chiaro nella flash (NVS non cifrata).
*/

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <WiFi.h>
#include <Preferences.h>

// ---------------- Configurazione pin ----------------
#define PIN_TFT_SCLK 12
#define PIN_TFT_MOSI 11
#define PIN_TFT_MISO 13
#define PIN_TFT_CS   10
#define PIN_TFT_DC   46
#define PIN_TFT_BL   45

#define PIN_TP_SDA   16
#define PIN_TP_SCL   15
#define PIN_TP_RST   18
#define PIN_TP_INT   17

#define CU_RX_PIN    44      // RX ESP32  <- TX CU
#define CU_TX_PIN    43      // TX ESP32  -> RX CU
#define CU_BAUD      19200

// ---------------- Display + touch ----------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341  _panel;
  lgfx::Bus_SPI        _bus;
  lgfx::Light_PWM      _light;
  lgfx::Touch_FT5x06   _touch;   // compatibile con FT6336
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = PIN_TFT_SCLK; c.pin_mosi = PIN_TFT_MOSI;
      c.pin_miso = PIN_TFT_MISO; c.pin_dc = PIN_TFT_DC;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = PIN_TFT_CS; c.pin_rst = -1;
      c.panel_width = 240; c.panel_height = 320;
      c.invert = true; c.rgb_order = true;          // BGR + inversione (setup Freenove)
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = PIN_TFT_BL; c.invert = false; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    { auto c = _touch.config();
      c.x_min = 0; c.x_max = 239; c.y_min = 0; c.y_max = 319;
      c.pin_int = PIN_TP_INT; c.pin_rst = PIN_TP_RST;
      c.bus_shared = false; c.offset_rotation = 0;
      c.i2c_port = 0; c.i2c_addr = 0x38;
      c.pin_sda = PIN_TP_SDA; c.pin_scl = PIN_TP_SCL; c.freq = 400000;
      _touch.config(c); _panel.setTouch(&_touch); }
    setPanel(&_panel);
  }
};
LGFX tft;

// ---------------- Stato generale ----------------
enum Screen { SCR_HOME, SCR_SETUP, SCR_SCAN };
Screen screen = SCR_HOME;

struct Rect {
  int16_t x, y, w, h;
  bool hit(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

const uint16_t C_BG   = TFT_BLACK;
const uint16_t C_OK   = TFT_GREEN;
const uint16_t C_WARN = TFT_ORANGE;
const uint16_t C_ERR  = TFT_RED;
const uint16_t C_DIM  = TFT_DARKGREY;
const uint16_t C_BTN  = 0x2965;   // grigio-blu scuro
const uint16_t C_KEY  = 0x39E7;
const uint16_t C_KEYS = 0x2104;   // tasti speciali

void drawButton(const Rect &r, const char *label, uint16_t bg, uint16_t fg = TFT_WHITE) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 5, bg);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(fg, bg);
  tft.setTextDatum(middle_center);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
}

// ---------------- Carrera Control Unit ----------------
enum CuState { CU_SEARCHING, CU_CONNECTED };
CuState  cuState   = CU_SEARCHING;
String   cuVersion = "";
String   lastRaw   = "";
uint32_t lastRxMs  = 0;
uint32_t lastTxMs  = 0;

const uint32_t POLL_MS    = 1000;
const uint32_t TIMEOUT_MS = 3500;

char carreraChecksum(const String &payload) {
  uint8_t s = 0;
  for (size_t i = 0; i < payload.length(); i++) s += (uint8_t)payload[i];
  return (char)('0' + (s & 0x0F));
}

void sendCmd(const String &payload) {
  Serial2.write('"');
  Serial2.print(payload);
  Serial2.write(carreraChecksum(payload));
  Serial2.write('$');
}

// true quando completa un frame '"' ... '$' (in `out` payload + checksum)
bool readFrame(String &out) {
  static String buf;
  static bool inFrame = false;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (lastRaw.length() >= 18) lastRaw.remove(0, 3);
    char h[4]; snprintf(h, sizeof h, "%02X ", b); lastRaw += h;
    if (b == '"') { buf = ""; inFrame = true; }
    else if (b == '$' && inFrame) { inFrame = false; out = buf; return true; }
    else if (inFrame && buf.length() < 64) buf += (char)b;
  }
  return false;
}

void cuTask(uint32_t now) {
  if (now - lastTxMs >= POLL_MS) {          // richiesta versione: rilevamento + keep-alive
    lastTxMs = now;
    sendCmd("0");
  }
  String f;
  while (readFrame(f)) {
    lastRxMs = now;
    if (f.length() >= 2) {
      String payload = f.substring(0, f.length() - 1);
      if (payload[0] == '0') cuVersion = payload.substring(1);
    }
    if (cuState != CU_CONNECTED) cuState = CU_CONNECTED;
  }
  if (cuState == CU_CONNECTED && now - lastRxMs > TIMEOUT_MS) {
    cuState = CU_SEARCHING;
    cuVersion = "";
  }
}

// ---------------- WiFi + persistenza ----------------
Preferences prefs;
String   savedSsid, savedPass;
volatile uint8_t lastReason = 0;

void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t r = info.wifi_sta_disconnected.reason;
    if (r != 8) lastReason = r;              // 8 = disconnessione richiesta da noi
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    lastReason = 0;
  }
}

String reasonText(uint8_t r) {
  switch (r) {
    case 201: return "rete non trovata";
    case 202: case 15: case 204: return "password errata?";
    case 200: return "segnale perso";
    default:  return String("errore ") + r;
  }
}

void loadCredentials() {
  prefs.begin("cdwifi", true);
  savedSsid = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  prefs.end();
}

void saveCredentials() {
  prefs.begin("cdwifi", false);
  prefs.putString("ssid", savedSsid);
  prefs.putString("pass", savedPass);
  prefs.end();
}

void startWifi() {
  if (savedSsid.isEmpty()) return;
  lastReason = 0;
  WiFi.disconnect();
  WiFi.begin(savedSsid.c_str(), savedPass.c_str());
}

void wifiInfo(String &l1, String &l2, uint16_t &col) {
  if (savedSsid.isEmpty()) { l1 = "Non configurato"; l2 = ""; col = C_DIM; }
  else if (WiFi.status() == WL_CONNECTED) { l1 = savedSsid; l2 = "IP " + WiFi.localIP().toString(); col = C_OK; }
  else if (lastReason) { l1 = savedSsid; l2 = reasonText(lastReason); col = C_ERR; }
  else { l1 = savedSsid; l2 = "Connessione..."; col = C_WARN; }
}

// ---------------- Schermata HOME ----------------
const Rect R_WIFI_BTN = {4, 274, 232, 40};
String homeSig;

void drawHome() {
  tft.fillScreen(C_BG);
  tft.setTextDatum(top_center);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.drawString("Carrera Digital", 120, 8);
  tft.drawFastHLine(10, 38, 220, C_DIM);

  // Control Unit
  bool ok = (cuState == CU_CONNECTED);
  uint16_t col = ok ? C_OK : C_WARN;
  tft.fillCircle(44, 85, 30, col);
  tft.setTextDatum(middle_center);
  tft.setTextColor(TFT_BLACK, col);
  tft.setFont(&fonts::FreeSansBold18pt7b);
  tft.drawString(ok ? "OK" : "..", 44, 85);

  tft.setTextDatum(top_left);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::Font2);
  tft.drawString("Control Unit", 90, 56);
  tft.setTextColor(col, C_BG);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.drawString(ok ? "CONNESSA" : "Ricerca...", 90, 74);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::Font2);
  tft.drawString(String("Versione: ") + (cuVersion.length() ? cuVersion : "-"), 90, 98);
  tft.drawString(String("Baud: ") + CU_BAUD, 90, 116);

  // WiFi
  String l1, l2; uint16_t wc;
  wifiInfo(l1, l2, wc);
  tft.drawFastHLine(10, 140, 220, C_DIM);
  tft.fillCircle(24, 176, 12, wc);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::Font2);
  tft.drawString("WiFi", 50, 152);
  tft.setTextColor(wc, C_BG);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.drawString(l1, 50, 170);
  tft.setFont(&fonts::Font2);
  tft.drawString(l2, 50, 194);

  if (!ok) {
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString(String("RX: ") + (lastRaw.length() ? lastRaw : "-"), 10, 240);
  }
  drawButton(R_WIFI_BTN, "Impostazioni WiFi", C_BTN);
}

void homeTask() {
  String l1, l2; uint16_t wc;
  wifiInfo(l1, l2, wc);
  String sig = String((int)cuState) + "|" + cuVersion + "|" + l1 + "|" + l2 + "|" + (cuState == CU_SEARCHING ? lastRaw : "");
  if (sig != homeSig) { homeSig = sig; drawHome(); }
}

// ---------------- Schermata SETUP (WiFi + tastiera) ----------------
const Rect R_BACK   = {2, 2, 54, 22};
const Rect R_SSID   = {4, 28, 186, 28};
const Rect R_PASS   = {4, 60, 186, 28};
const Rect R_SCAN   = {194, 28, 42, 28};
const Rect R_SHOW   = {194, 60, 42, 28};
const Rect R_SAVE   = {4, 92, 150, 30};
const Rect R_FORGET = {158, 92, 78, 30};

String edSsid, edPass;
int    focus    = 0;      // 0 = SSID, 1 = password
bool   showPass = false;
bool   shiftOn  = false;
int    symPage  = 0;      // 0 lettere, 1 numeri/simboli, 2 altri simboli
String setupMsg = "";
uint16_t setupMsgCol = TFT_WHITE;
bool   connecting = false;
uint32_t connStart = 0;

enum KType : uint8_t { K_CHAR, K_SHIFT, K_PAGE, K_DEL, K_MODE, K_SPACE, K_ENTER };
struct Key { Rect r; char c; KType t; };
Key keys[40];
int nKeys = 0;
const int KY = 156, KH = 40, KW = 24;

void buildKeys() {
  const char *r1, *r2, *r3 = ".,:;!?'";
  if (symPage == 0)      { r1 = "qwertyuiop"; r2 = "asdfghjkl"; r3 = "zxcvbnm"; }
  else if (symPage == 1) { r1 = "1234567890"; r2 = "@#$%&*-_=+"; }
  else                   { r1 = "()[]{}<>/\\"; r2 = "|~`^\""; }

  nKeys = 0;
  auto add = [&](int x, int row, int w, char c, KType t) {
    keys[nKeys++] = { {(int16_t)x, (int16_t)(KY + row * KH), (int16_t)w, (int16_t)KH}, c, t };
  };
  auto addRow = [&](const char *s, int row, int x0) {
    for (int i = 0; s[i]; i++) {
      char c = s[i];
      if (symPage == 0 && shiftOn) c = toupper(c);
      add(x0 + i * KW, row, KW, c, K_CHAR);
    }
  };
  addRow(r1, 0, (240 - strlen(r1) * KW) / 2);
  addRow(r2, 1, (240 - strlen(r2) * KW) / 2);
  add(0, 2, 36, 0, symPage == 0 ? K_SHIFT : K_PAGE);
  addRow(r3, 2, 36);
  add(204, 2, 36, 0, K_DEL);
  add(0, 3, 54, 0, K_MODE);
  add(54, 3, 132, ' ', K_SPACE);
  add(186, 3, 54, 0, K_ENTER);
}

void drawKeyboard() {
  buildKeys();
  tft.fillRect(0, KY, 240, 4 * KH, C_BG);
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(middle_center);
  for (int i = 0; i < nKeys; i++) {
    const Key &k = keys[i];
    uint16_t bg = (k.t == K_CHAR || k.t == K_SPACE) ? C_KEY : C_KEYS;
    if (k.t == K_SHIFT && shiftOn) bg = TFT_BLUE;
    if (k.t == K_ENTER) bg = TFT_DARKGREEN;
    tft.fillRoundRect(k.r.x + 1, k.r.y + 2, k.r.w - 2, k.r.h - 4, 4, bg);
    tft.setTextColor(TFT_WHITE, bg);
    char buf[2] = {k.c, 0};
    const char *label = buf;
    switch (k.t) {
      case K_SHIFT: label = "Shf"; break;
      case K_PAGE:  label = (symPage == 1) ? "#+=" : "123"; break;
      case K_DEL:   label = "Del"; break;
      case K_MODE:  label = (symPage == 0) ? "123" : "abc"; break;
      case K_SPACE: label = "spazio"; break;
      case K_ENTER: label = (focus == 0) ? "Avanti" : "Vai"; break;
      default: break;
    }
    tft.drawString(label, k.r.x + k.r.w / 2, k.r.y + k.r.h / 2);
  }
}

void drawField(const Rect &r, const String &val, bool masked, bool focused, const char *placeholder) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 4, C_BG);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 4, focused ? TFT_CYAN : C_DIM);
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(middle_left);
  String s;
  if (masked) { for (size_t i = 0; i < val.length(); i++) s += '*'; } else s = val;
  if (focused) s += '_';
  if (s.isEmpty() || (s == "_")) {
    tft.setTextColor(C_DIM, C_BG);
    tft.drawString(focused ? "_" : placeholder, r.x + 6, r.y + r.h / 2);
    return;
  }
  while (s.length() > 1 && tft.textWidth(s) > r.w - 12) s.remove(0, 1);   // mostra la coda
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.drawString(s, r.x + 6, r.y + r.h / 2);
}

void drawFields() {
  drawField(R_SSID, edSsid, false, focus == 0, "Nome rete (SSID)");
  drawField(R_PASS, edPass, !showPass, focus == 1, "Password");
}

void drawSetupMsg() {
  tft.fillRect(0, 124, 240, 30, C_BG);
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(middle_left);
  tft.setTextColor(setupMsgCol, C_BG);
  tft.drawString(setupMsg, 6, 139);
}

void setMsg(const String &m, uint16_t col) {
  setupMsg = m; setupMsgCol = col;
  if (screen == SCR_SETUP) drawSetupMsg();
}

void drawSetup() {
  tft.fillScreen(C_BG);
  tft.setFont(&fonts::Font2);
  drawButton(R_BACK, "< Esci", C_BTN);
  tft.setTextDatum(middle_left);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.drawString("Impostazioni WiFi", 62, 13);
  drawFields();
  drawButton(R_SCAN, "Scan", C_BTN);
  drawButton(R_SHOW, showPass ? "Hide" : "Show", C_BTN);
  drawButton(R_SAVE, "Salva e connetti", TFT_DARKGREEN);
  drawButton(R_FORGET, "Elimina", 0x8000);
  drawSetupMsg();
  drawKeyboard();
}

void enterSetup() {
  edSsid = savedSsid; edPass = savedPass;
  focus = savedSsid.isEmpty() ? 0 : 1;
  showPass = false; shiftOn = false; symPage = 0; connecting = false;
  setupMsg = savedSsid.isEmpty() ? "Scegli una rete o scrivi l'SSID" : "Modifica e premi Salva";
  setupMsgCol = C_DIM;
  screen = SCR_SETUP;
  drawSetup();
}

void leaveSetup() {
  connecting = false;
  if (!savedSsid.isEmpty() && WiFi.status() != WL_CONNECTED) startWifi();   // dopo scan/modifiche
  screen = SCR_HOME;
  homeSig = "";
}

void saveAndConnect() {
  if (edSsid.isEmpty()) { setMsg("Inserisci il nome della rete", C_ERR); return; }
  savedSsid = edSsid; savedPass = edPass;
  saveCredentials();
  startWifi();
  connecting = true; connStart = millis();
  setMsg("Salvato. Connessione...", C_WARN);
}

void forgetCredentials() {
  prefs.begin("cdwifi", false);
  prefs.clear();
  prefs.end();
  savedSsid = ""; savedPass = ""; edSsid = ""; edPass = "";
  connecting = false; lastReason = 0;
  WiFi.disconnect();
  focus = 0;
  drawFields();
  setMsg("Credenziali eliminate", C_WARN);
}

void typeChar(char c) {
  String &f = (focus == 0) ? edSsid : edPass;
  if (f.length() >= (focus == 0 ? 32u : 63u)) return;
  f += c;
  if (shiftOn && symPage == 0) { shiftOn = false; drawKeyboard(); }
  drawFields();
}

void deleteChar() {
  String &f = (focus == 0) ? edSsid : edPass;
  if (f.length()) { f.remove(f.length() - 1); drawFields(); }
}

void startScan();

void setupTap(int x, int y, bool &isDel) {
  isDel = false;
  if (R_BACK.hit(x, y))   { leaveSetup(); return; }
  if (R_SSID.hit(x, y))   { focus = 0; drawFields(); drawKeyboard(); return; }
  if (R_PASS.hit(x, y))   { focus = 1; drawFields(); drawKeyboard(); return; }
  if (R_SHOW.hit(x, y))   { showPass = !showPass; drawButton(R_SHOW, showPass ? "Hide" : "Show", C_BTN); drawFields(); return; }
  if (R_SAVE.hit(x, y))   { saveAndConnect(); return; }
  if (R_FORGET.hit(x, y)) { forgetCredentials(); return; }
  if (R_SCAN.hit(x, y))   { screen = SCR_SCAN; startScan(); return; }
  for (int i = 0; i < nKeys; i++) {
    const Key &k = keys[i];
    if (!k.r.hit(x, y)) continue;
    switch (k.t) {
      case K_CHAR:
      case K_SPACE: typeChar(k.c); break;
      case K_SHIFT: shiftOn = !shiftOn; drawKeyboard(); break;
      case K_PAGE:  symPage = (symPage == 1) ? 2 : 1; drawKeyboard(); break;
      case K_MODE:  symPage = (symPage == 0) ? 1 : 0; shiftOn = false; drawKeyboard(); break;
      case K_DEL:   deleteChar(); isDel = true; break;
      case K_ENTER:
        if (focus == 0) { focus = 1; drawFields(); drawKeyboard(); }
        else saveAndConnect();
        break;
    }
    return;
  }
}

void setupTask() {
  if (!connecting) return;
  if (WiFi.status() == WL_CONNECTED) {
    connecting = false;
    setMsg("Connesso! IP " + WiFi.localIP().toString(), C_OK);
  } else if (lastReason == 201 || lastReason == 202 || lastReason == 15 || lastReason == 204) {
    connecting = false;
    setMsg("Fallito: " + reasonText(lastReason), C_ERR);
  } else if (millis() - connStart > 20000) {
    connecting = false;
    setMsg("Timeout di connessione", C_ERR);
  }
}

// ---------------- Schermata SCAN ----------------
const int MAX_NETS = 7;
String   netName[MAX_NETS];
int      netRssi[MAX_NETS];
bool     netLock[MAX_NETS];
int      nNets = 0;
const Rect R_SCAN_BACK  = {4, 276, 112, 38};
const Rect R_SCAN_AGAIN = {124, 276, 112, 38};

void drawScanList() {
  tft.fillScreen(C_BG);
  tft.setTextDatum(middle_left);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.drawString("Reti WiFi", 8, 13);
  tft.setFont(&fonts::Font2);
  if (nNets == 0) {
    tft.setTextColor(C_ERR, C_BG);
    tft.drawString("Nessuna rete trovata", 8, 60);
  }
  for (int i = 0; i < nNets; i++) {
    int y = 30 + i * 34;
    tft.fillRoundRect(4, y, 232, 32, 4, C_BTN);
    String n = netName[i];
    while (n.length() > 1 && tft.textWidth(n) > 150) n.remove(n.length() - 1);
    tft.setTextColor(TFT_WHITE, C_BTN);
    tft.setTextDatum(middle_left);
    tft.drawString(n, 10, y + 16);
    tft.setTextDatum(middle_right);
    tft.drawString(String(netRssi[i]) + (netLock[i] ? " *" : ""), 230, y + 16);
  }
  drawButton(R_SCAN_BACK, "Indietro", C_BTN);
  drawButton(R_SCAN_AGAIN, "Riscansiona", C_BTN);
}

void startScan() {
  tft.fillScreen(C_BG);
  tft.setTextDatum(middle_center);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.drawString("Scansione in corso...", 120, 150);

  WiFi.disconnect();                       // la scansione fallisce spesso durante i tentativi di connessione
  int n = WiFi.scanNetworks(false, false);
  nNets = 0;
  for (int i = 0; i < n && nNets < MAX_NETS; i++) {
    String s = WiFi.SSID(i);
    if (s.isEmpty()) continue;
    bool dup = false;
    for (int j = 0; j < nNets; j++) if (netName[j] == s) dup = true;
    if (dup) continue;
    netName[nNets] = s;
    netRssi[nNets] = WiFi.RSSI(i);
    netLock[nNets] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    nNets++;
  }
  WiFi.scanDelete();
  drawScanList();
}

void scanTap(int x, int y) {
  if (R_SCAN_BACK.hit(x, y))  { screen = SCR_SETUP; drawSetup(); return; }
  if (R_SCAN_AGAIN.hit(x, y)) { startScan(); return; }
  for (int i = 0; i < nNets; i++) {
    if (Rect{4, (int16_t)(30 + i * 34), 232, 32}.hit(x, y)) {
      edSsid = netName[i]; edPass = ""; focus = 1;
      setupMsg = netLock[i] ? "Inserisci la password" : "Rete aperta: lascia vuota la password";
      setupMsgCol = C_DIM;
      screen = SCR_SETUP; drawSetup();
      return;
    }
  }
}

// ---------------- Touch ----------------
void touchTask(uint32_t now) {
  static bool wasDown = false, delHeld = false;
  static uint32_t downMs = 0, lastRepeat = 0;
  int32_t x, y;
  bool down = tft.getTouch(&x, &y);

  if (down && !wasDown) {
    downMs = now; lastRepeat = now; delHeld = false;
    switch (screen) {
      case SCR_HOME:
        if (R_WIFI_BTN.hit(x, y)) enterSetup();
        break;
      case SCR_SETUP: { bool d; setupTap(x, y, d); delHeld = d; break; }
      case SCR_SCAN:  scanTap(x, y); break;
    }
  } else if (down && delHeld && screen == SCR_SETUP && now - downMs > 500 && now - lastRepeat > 70) {
    lastRepeat = now;
    deleteChar();                           // ripetizione del tasto Del
  }
  wasDown = down;
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0);
  tft.setBrightness(200);

  Serial2.begin(CU_BAUD, SERIAL_8N1, CU_RX_PIN, CU_TX_PIN);

  WiFi.persistent(false);                   // le credenziali le gestiamo noi in NVS
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWiFiEvent);
  loadCredentials();
  startWifi();

  drawHome();
}

void loop() {
  uint32_t now = millis();
  cuTask(now);
  touchTask(now);
  if (screen == SCR_HOME) homeTask();
  else if (screen == SCR_SETUP) setupTask();
}
