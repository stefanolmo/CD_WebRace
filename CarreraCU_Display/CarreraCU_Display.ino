/*
  Carrera Digital 124/132 Control Unit -> Freenove ESP32-S3 CYD 2.8" (240x320)

  Collegamento seriale (UART1) alla Control Unit e visualizzazione dello stato
  della connessione sul display.

  Librerie (Library Manager): LovyanGFX
  Board: ESP32S3 Dev Module, "USB CDC On Boot: Enabled", PSRAM come da scheda.

  !!! DA VERIFICARE PRIMA DI COLLEGARE !!!
  1) Pin display: verificati sullo schema ufficiale Freenove 2.8" (FNK0104B,
     ILI9341): CS=10, MOSI=11, SCK=12, MISO=13, DC(LCD_RS)=46, BL=45.
  2) Pin CU_RX / CU_TX: connettore P2 ("UART" nello schema) = GPIO44 (RXD0) e
     GPIO43 (TXD0). L'USB della scheda e' nativa, quindi questi pin sono liberi.
     Attenzione: la ROM scrive i log di boot su TXD0 a 115200 baud, quindi
     all'accensione la CU puo' ricevere qualche byte spurio.
     Mantenere "USB CDC On Boot: Enabled" cosi' Serial resta sull'USB nativa.
  3) Livelli logici: l'ESP32 e' a 3.3 V. Misurare con un tester i livelli della
     porta della CU; se sono a 5 V serve un level shifter. Non collegare a occhio.
  4) Protocollo (19200 8N1, frame '"' ... '$', checksum 1 char): ricostruito
     dal progetto open source "carreralib". Se non si connette, guardare la
     riga "RX" sul display: mostra i byte grezzi ricevuti.
*/

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ---------------- Configurazione pin ----------------
// Display (FNK0104B, da schema ufficiale Freenove)
#define PIN_TFT_SCLK 12
#define PIN_TFT_MOSI 11
#define PIN_TFT_MISO 13
#define PIN_TFT_CS   10
#define PIN_TFT_DC   46
#define PIN_TFT_BL   45
#define TFT_INVERT   false   // se i colori sono invertiti metti true

// UART verso la Control Unit
#define CU_RX_PIN    44      // RX ESP32  <- TX CU
#define CU_TX_PIN    43      // TX ESP32  -> RX CU
#define CU_BAUD      19200

// ---------------- Display ----------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
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
      c.invert = TFT_INVERT; c.rgb_order = false;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = PIN_TFT_BL; c.invert = false; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
LGFX tft;

// ---------------- Protocollo Carrera ----------------
enum State { SEARCHING, CONNECTED };
State state = SEARCHING;

String   version   = "";
String   lastRaw   = "";      // ultimi byte ricevuti (debug)
bool     lastChkOk = false;
uint32_t lastRxMs  = 0;
uint32_t lastTxMs  = 0;
uint32_t frames    = 0;

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

// Ritorna true quando completa un frame '"' ... '$' (in `out` payload + checksum)
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

// ---------------- UI ----------------
void drawStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(top_center);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.drawString("Carrera Digital", 120, 12);
  tft.drawFastHLine(10, 44, 220, TFT_DARKGREY);
}

void drawStatus() {
  bool ok = (state == CONNECTED);
  uint16_t col = ok ? TFT_GREEN : TFT_ORANGE;

  tft.fillRect(0, 60, 240, 150, TFT_BLACK);
  tft.fillCircle(120, 110, 36, col);
  tft.setTextDatum(middle_center);
  tft.setTextColor(TFT_BLACK, col);
  tft.setFont(&fonts::FreeSansBold24pt7b);
  tft.drawString(ok ? "OK" : "...", 120, 110);

  tft.setTextDatum(top_center);
  tft.setTextColor(col, TFT_BLACK);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.drawString(ok ? "CONNESSO" : "Ricerca Control Unit", 120, 165);

  tft.fillRect(0, 210, 240, 110, TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(top_left);
  tft.drawString(String("Baud: ") + CU_BAUD, 10, 215);
  tft.drawString(String("Versione CU: ") + (version.length() ? version : "-"), 10, 235);
  tft.drawString(String("Frame: ") + frames + (lastChkOk ? "  chk OK" : "  chk ?"), 10, 255);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(String("RX: ") + (lastRaw.length() ? lastRaw : "-"), 10, 280);
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.setBrightness(200);
  drawStatic();
  drawStatus();

  Serial2.begin(CU_BAUD, SERIAL_8N1, CU_RX_PIN, CU_TX_PIN);
}

void loop() {
  uint32_t now = millis();
  bool dirty = false;

  // Richiesta versione ("0") periodica: serve sia per rilevare la CU sia da keep-alive
  if (now - lastTxMs >= POLL_MS) {
    lastTxMs = now;
    sendCmd("0");
  }

  String f;
  while (readFrame(f)) {
    frames++;
    lastRxMs = now;
    // f = payload + 1 char di checksum
    if (f.length() >= 2) {
      String payload = f.substring(0, f.length() - 1);
      lastChkOk = (f[f.length() - 1] == carreraChecksum(payload));
      if (payload[0] == '0') version = payload.substring(1);   // risposta a "0"
    }
    if (state != CONNECTED) { state = CONNECTED; Serial.println("CU connessa"); }
    dirty = true;
  }

  if (state == CONNECTED && now - lastRxMs > TIMEOUT_MS) {
    state = SEARCHING; version = ""; frames = 0;
    Serial.println("CU persa");
    dirty = true;
  }

  static uint32_t lastDraw = 0;
  if (dirty || now - lastDraw > 500) { lastDraw = now; drawStatus(); }
}
