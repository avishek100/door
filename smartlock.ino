

/*
 * ================================================================
 *   ESP32 Smart Door Lock — ILI9341 2.8" TFT + XPT2046 Touch
 *   + MFRC522 RFID + Solenoid Lock via Relay
 *
 *  Features:
 *    - RFID card scan → name lookup → grant/deny
 *    - ILI9341 TFT feedback screen with touch menu
 *    - Touch UI: Admin panel (add/block card, view log)
 *    - Solenoid lock control via relay
 *    - WiFi + web-based Smart Register dashboard
 *
 *  Libraries (install via Arduino Library Manager):
 *    1. MFRC522             by GithubCommunity
 *    2. Adafruit ILI9341    by Adafruit
 *    3. Adafruit GFX        by Adafruit
 *    4. XPT2046_Touchscreen by Paul Stoffregen
 *    5. WiFi                (built-in ESP32)
 *    6. WebServer           (built-in ESP32)
 * ================================================================
 *
 *  PIN MAP — ILI9341 2.8" with XPT2046 touch
 *
 *  TFT/Display (HSPI)      ESP32
 *  ------------------      -----
 *  VCC                 →   3.3V
 *  GND                 →   GND
 *  CS                  →   GPIO 15
 *  RESET               →   GPIO 25
 *  DC (A0)             →   GPIO 12
 *  SDI (MOSI)          →   GPIO 13
 *  SCK                 →   GPIO 14
 *  LED (backlight)     →   3.3V (always on) or GPIO for PWM
 *
 *  XPT2046 Touch (shares HSPI)
 *  ------------------      -----
 *  T_CLK               →   GPIO 14  (shared with TFT SCK)
 *  T_CS                →   GPIO 22  (separate!)
 *  T_DIN               →   GPIO 13  (shared with TFT MOSI)
 *  T_DO                →   GPIO 27  (separate MISO for touch)
 *  T_IRQ               →   GPIO 21  (interrupt)
 *
 *  MFRC522 RFID (VSPI)     ESP32
 *  ------------------      -----
 *  SDA (SS)            →   GPIO 5
 *  SCK                 →   GPIO 18
 *  MOSI                →   GPIO 23
 *  MISO                →   GPIO 19
 *  RST                 →   GPIO 4
 *  3.3V                →   3.3V
 *  GND                 →   GND
 *
 *  Relay Module            ESP32
 *  ------------------      -----
 *  IN                  →   GPIO 16
 *  VCC                 →   3.3V or 5V  ← Works with 3.3V-5V depending on relay board
 *  GND                 →   GND ← shared GND with ESP32
 *
 *  Solenoid Lock           Relay & Power
 *  ------------------      -------------
 *  + (Red)             →   Relay NO  (Normally Open terminal)
 *  - (Black)           →   12V PSU GND  ← CRITICAL: must connect to PSU GND
 *  12V PSU +           →   Relay COM (Common terminal)
 *  12V PSU GND         →   Solenoid - (Black) AND ESP32 GND (common ground)
 *
 *  ⚠️  WIRING CHECKLIST — if solenoid doesn't fire:
 *  1. Relay VCC can be 3.3V or 5V — depends on your relay board (works with 3.8V here)
 *  2. Solenoid - (black) wire must go to 12V PSU GND
 *  3. 12V PSU GND and ESP32 GND must share a common ground
 *  4. Confirm you are using the NO terminal (not NC) on the relay
 *  5. At startup, Serial Monitor prints relay diagnostic — listen for click
 *
 * ================================================================
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <time.h>
#include <ArduinoOTA.h>

// ── WiFi ──────────────────────────────────────────────────────
const char *WIFI_SSID = "Bikash_DHInternet";
const char *WIFI_PASSWORD = "DHInternet@20992086";

// ── Pins ──────────────────────────────────────────────────────
// RFID — VSPI (default SPI)
#define RFID_SS 5
#define RFID_RST 4
#define RFID_MOSI 23
#define RFID_MISO 19
#define RFID_SCK 18

// TFT — HSPI
#define TFT_CS 15
#define TFT_RST 25
#define TFT_DC 12
#define TFT_SCK 14
#define TFT_MOSI 13

// Touch — shares HSPI SCK/MOSI, has own CS and MISO
#define TOUCH_CS 22
#define TOUCH_IRQ 21
#define TOUCH_MISO 27 // T_DO pin

// Buzzer
#define BUZZER_PIN 26
#define BUZZER_FREQ 2000
#define BUZZER_MS 40

// Relay
#define RELAY_PIN 16

// Exit button — physical touch button to open the door from outside
// Use a GPIO that supports INPUT_PULLUP; GPIO 34 cannot use internal pull-ups.
#define EXIT_BUTTON_PIN 32
#define BUTTON_DEBOUNCE_MS 60
#define EXIT_BUTTON_STABLE_MS 30

// Screen sleep & backlight PWM control
#define TFT_LED_PIN 33           // Backlight control via LEDC PWM (GPIO 33)
#define LEDC_CHANNEL 0           // LEDC channel for PWM
#define LEDC_FREQ 5000           // PWM frequency 5kHz
#define LEDC_RESOLUTION 8        // 8-bit resolution (0-255)
#define BACKLIGHT_BRIGHTNESS 255 // Max brightness when on

// Outside unlock button (second touch button) — GPIO for future use
#define OUTSIDE_TOUCH_PIN 35 // GPIO 35 for outside touch unlock button (optional)

#define TIMEZONE_UTC_OFFSET_SECONDS 20700

// ── RELAY LOGIC CONFIGURATION ─────────────────────────────────
// Most relay breakout boards (blue/green PCB with optocoupler) are
// ACTIVE-LOW: the relay ENERGIZES (clicks ON) when IN pin is LOW.
// When energized, COM connects to NO — which powers your solenoid.
//
// If your solenoid still won't fire after confirming wiring:
//   → Try swapping: #define LOCK_OPEN HIGH  / #define LOCK_CLOSE LOW
//   (some relay boards have onboard logic that inverts the signal)
#define LOCK_OPEN LOW   // LOW  = relay energized = COM→NO = solenoid ON
#define LOCK_CLOSE HIGH // HIGH = relay de-energized = COM→NC = solenoid OFF

// ── Timing ────────────────────────────────────────────────────
#define UNLOCK_MS 5000      // Door open duration (5 seconds)
#define MSG_MS 3000         // Result screen duration for quick UI testing
#define TOUCH_DEBOUNCE 200  // ms between touch events while pressed
#define TOUCH_Z_THRESHOLD 0 // Minimum pressure for valid touch
#define RFID_SCAN_TIMEOUT 100
#define DISPLAY_UPDATE_RATE 20 // ms between display updates // RFID polling timeout
#define SCREENSAVER_MS 15000   // Inactivity before screen saver (15 seconds)
// ── Touch Calibration Values ──────────────────────────────
// PORTRAIT MODE - Calibrated with corner touches
// Top-left raw: x=3734, y=3915 → screen (0, 0)
// Bottom-right raw: x=592, y=500 → screen (239, 319)
#define TOUCH_X_MIN 592
#define TOUCH_X_MAX 3734
#define TOUCH_Y_MIN 500
#define TOUCH_Y_MAX 3915
#define SCREEN_W 240
#define SCREEN_H 320

// ── Navigation Bar ───────────────────────────────────────────────
#define NAV_HEIGHT 28
#define NAV_Y (SCREEN_H - NAV_HEIGHT) // 292
#define CONTENT_H (NAV_Y - 36)        // 256 (below header, above nav)

// ── Colors (RGB565) — Vibrant & Saturated ─────────────────────
#define C_BG 0x0000      // Black background
#define C_PANEL 0x2945   // Dark teal
#define C_PANEL2 0x18C3  // Deep navy panel
#define C_WHITE 0xFFFF   // Pure white
#define C_GREEN 0x07E0   // Bright lime green
#define C_DKGREEN 0x0400 // Dark green
#define C_RED 0xF800     // Bright red
#define C_BLUE 0x001F    // Bright blue
#define C_LBLUE 0x049F   // Light bright blue
#define C_YELLOW 0xFFE0  // Bright yellow
#define C_CYAN 0x07FF    // Bright cyan
#define C_ORANGE 0xFE60  // Bright orange
#define C_GRAY 0x8410    // Medium gray
#define C_DKGRAY 0x4208  // Dark gray
#define C_PURPLE 0xF81F  // Bright magenta/purple
#define C_HEADER 0x00FF  // Bright blue for header
#define C_TEXT 0xC618    // Light gray text
#define C_MUTED 0x7BEF   // Soft muted text

// ── Admin PIN ─────────────────────────────────────────────────
const String ADMIN_PIN = "2424";
const String WEB_PASSWORD = "2424"; // change this to protect web open-door access

// ── Card database ─────────────────────────────────────────────
#define MAX_CARDS 20
struct Card
{
  String uid;
  String name;
  String role;
  bool blocked;
};

Card cards[MAX_CARDS] = {};
int cardCount = 0;

// ── Calibration test data ────────────────────────────────────
struct CalibPoint
{
  int screen_x, screen_y; // Target screen coordinates
  int raw_x, raw_y;       // Captured raw touch coordinates
  bool captured;
};

CalibPoint calibPoints[4] = {
    {10, 45},  // Top-left (in header area)
    {230, 45}, // Top-right
    {10, 310}, // Bottom-left
    {230, 310} // Bottom-right
};
int calibIdx = 0;

// ── Access log (circular, last 50 entries) ────────────────────
#define MAX_LOG 200
struct LogEntry
{
  String uid, name, role, ts;
  bool granted;
};
LogEntry logBuf[MAX_LOG];
int logCount = 0, logHead = 0;

// ── Screen states ─────────────────────────────────────────────
enum Screen
{
  SCR_IDLE,
  SCR_GRANTED,
  SCR_DENIED,
  SCR_BLOCKED,
  SCR_ADMIN_PIN,
  SCR_ADMIN_MENU,
  SCR_VIEW_LOG,
  SCR_SCAN_NEW,
  SCR_EDIT_CARD,
  SCR_BLOCK_CARD,
  SCR_LOG_WEB,
  SCR_CALIBRATE
};
Screen currentScreen = SCR_IDLE;

// ── Hardware objects ──────────────────────────────────────────
MFRC522 rfid(RFID_SS, RFID_RST);

SPIClass hSPI(HSPI);
Adafruit_ILI9341 tft(&hSPI, TFT_DC, TFT_CS, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

WebServer server(80);

// ── State ─────────────────────────────────────────────────────
// ── State for UI messaging ───────────────────────────────────
unsigned long lastTouch = 0;
bool touchWasDown = false;
int lastRawTouchX = 0;
int lastRawTouchY = 0;
unsigned long lastRFIDScan = 0;
unsigned long lastActivity = 0;
unsigned long messageTimeout = 0;
String adminPinEntry = "";
bool waitingForNewCard = false;
bool waitingForEditCard = false;
bool waitingForBlockCard = false;
bool editExistingMode = false;
String editCardUid = "";
String editCardName = "";
String editCardRole = "";
int editCardField = 0; // 0 = Name, 1 = Role
int editCardIndex = -1;
int blockCardIndex = -1;
int logPage = 0;
bool screenSleeping = false;
String lastGrantedName = "";
String lastGrantedRole = "";
String lastDeniedUID = "";
bool showingMessage = false;
String messageText = "";

// ── RFID debounce to prevent duplicate logs ─────────────────
String lastScannedUID = "";          // Last card that was in range
unsigned long lastCardLeftRange = 0; // When card left the reader
#define CARD_DEBOUNCE_MS 1000        // Must wait 1 sec before same card logs again

bool doorUnlocked = false;
unsigned long unlockUntil = 0;
int exitButtonLastRawState = HIGH;
int exitButtonLastStableState = HIGH;
unsigned long exitButtonDebounceTime = 0;
unsigned long lastExitButtonLog = 0;     // Prevent duplicate exit button logs
#define EXIT_BUTTON_DEBOUNCE_LOG_MS 2000 // Must wait 2 sec before exit button logs again

// unsigned long lastButtonPress = 0; // Removed, simplified logic

// bool exitButtonReady = false; // Removed, simplified logic

bool firstTouchAfterWake = false; // Flag: consume first touch after waking
bool backlightForced = false;     // Force backlight on during update
String lastBlockedName = "";

// =================================================================
//  RELAY CONTROL — centralised so every unlock goes through here
// =================================================================

// ── relayDiagnostic ───────────────────────────────────────────
// Called once at boot. Pulses the relay so you can hear it click
// and confirms GPIO state is readable. Results printed to Serial.
void relayDiagnostic()
{
  Serial.println("\n========================================");
  Serial.println("  RELAY DIAGNOSTIC (startup pulse test)");
  Serial.println("========================================");
  Serial.printf("  RELAY_PIN   : GPIO %d\n", RELAY_PIN);
  Serial.printf("  LOCK_OPEN   : %s (relay energized = solenoid ON)\n", LOCK_OPEN == LOW ? "LOW" : "HIGH");
  Serial.printf("  LOCK_CLOSE  : %s (relay de-energized = solenoid OFF)\n", LOCK_CLOSE == LOW ? "LOW" : "HIGH");

  // Confirm initial state (should be LOCK_CLOSE)
  Serial.printf("  Initial GPIO state: %s\n", digitalRead(RELAY_PIN) == LOW ? "LOW" : "HIGH");

  // Pulse 1: energize (should click ON)
  Serial.println("  Pulsing relay ON  (listen for click)...");
  digitalWrite(RELAY_PIN, LOCK_OPEN);
  delay(200);
  Serial.printf("  GPIO after LOCK_OPEN : %s\n", digitalRead(RELAY_PIN) == LOW ? "LOW" : "HIGH");

  // Pulse 2: de-energize (should click OFF)
  Serial.println("  Pulsing relay OFF (listen for click)...");
  digitalWrite(RELAY_PIN, LOCK_CLOSE);
  delay(200);
  Serial.printf("  GPIO after LOCK_CLOSE: %s\n", digitalRead(RELAY_PIN) == LOW ? "LOW" : "HIGH");

  Serial.println("----------------------------------------");
  Serial.println("  If you heard TWO clicks → relay wiring OK.");
  Serial.println("  If solenoid still doesn't move, check:");
  Serial.println("  1. Relay VCC connected to 5V (not 3.3V)");
  Serial.println("  2. Solenoid (-) connected to 12V PSU GND");
  Serial.println("  3. 12V PSU GND shared with ESP32 GND");
  Serial.println("  4. Using NO terminal (not NC) on relay");
  Serial.println("  5. Try swapping LOCK_OPEN/LOCK_CLOSE defines");
  Serial.println("========================================\n");
}

// ── unlockDoor / lockDoor ─────────────────────────────────────
// Centralised relay calls with Serial confirmation every time.
void unlockDoor(const char *reason)
{
  digitalWrite(RELAY_PIN, LOCK_OPEN);
  delay(20); // small settle delay before reading back
  int state = digitalRead(RELAY_PIN);
  Serial.printf("[RELAY] UNLOCK  (%s) → GPIO%d = %s %s\n",
                reason, RELAY_PIN,
                state == LOW ? "LOW" : "HIGH",
                state == LOCK_OPEN ? "✓ CORRECT" : "✗ MISMATCH — check wiring");
}

void lockDoor(const char *reason)
{
  digitalWrite(RELAY_PIN, LOCK_CLOSE);
  delay(20);
  int state = digitalRead(RELAY_PIN);
  Serial.printf("[RELAY] LOCK    (%s) → GPIO%d = %s %s\n",
                reason, RELAY_PIN,
                state == HIGH ? "HIGH" : "LOW",
                state == LOCK_CLOSE ? "✓ CORRECT" : "✗ MISMATCH — check wiring");
}

// =================================================================
//  BACKLIGHT CONTROL — LEDC PWM for smartphone-style sleep
// =================================================================

// ── backlightOn ─────────────────────────────────────────────────
// Turn backlight on to full brightness via PWM
void backlightOn(const char *reason = "user activity")
{
  if (!backlightForced)
  {
    ledcWrite(TFT_LED_PIN, BACKLIGHT_BRIGHTNESS);
    screenSleeping = false;
    lastActivity = millis();
    Serial.printf("[BACKLIGHT] ON (%s)\n", reason);
  }
}

// ── backlightOff ────────────────────────────────────────────────
// Turn backlight off completely (PWM = 0, true blackout)
void backlightOff(const char *reason = "inactivity")
{
  if (!backlightForced)
  {
    ledcWrite(TFT_LED_PIN, 0);
    screenSleeping = true;
    Serial.printf("[BACKLIGHT] OFF (%s)\n", reason);
  }
}

// ── backlightSet ────────────────────────────────────────────────
// Set backlight to specific brightness (0-255)
void backlightSet(uint8_t brightness, const char *reason = "pwm control")
{
  if (!backlightForced)
  {
    ledcWrite(TFT_LED_PIN, brightness);
    if (brightness > 0)
      screenSleeping = false;
    Serial.printf("[BACKLIGHT] SET %d/255 (%s)\n", brightness, reason);
  }
}

// ── setBacklightForced ──────────────────────────────────────────
// Force backlight on (for updates), cannot be turned off
void setBacklightForced(bool forced)
{
  backlightForced = forced;
  if (forced)
  {
    ledcWrite(TFT_LED_PIN, BACKLIGHT_BRIGHTNESS);
    screenSleeping = false;
    Serial.println("[BACKLIGHT] FORCED ON (update or maintenance)");
  }
  else
  {
    Serial.println("[BACKLIGHT] Forced mode released");
  }
}

// =================================================================
//  HELPERS
// =================================================================
String getUID()
{
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++)
  {
    if (rfid.uid.uidByte[i] < 0x10)
      uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

int findCard(const String &uid)
{
  for (int i = 0; i < cardCount; i++)
    if (cards[i].uid.equalsIgnoreCase(uid))
      return i;
  return -1;
}

String getCurrentTimestamp()
{
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1000))
  {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
  }

  unsigned long seconds = millis() / 1000;
  unsigned long h = (seconds / 3600) % 24;
  unsigned long m = (seconds / 60) % 60;
  unsigned long s = seconds % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}

void loadEditCard(int idx)
{
  if (idx < 0 || idx >= cardCount)
    return;

  editCardIndex = idx;
  editCardUid = cards[idx].uid;
  editCardName = cards[idx].name;
  editCardRole = cards[idx].role;
  editCardField = 0;
  currentScreen = SCR_EDIT_CARD;
  drawEditCardScreen();
}

void loadBlockCard(int idx)
{
  if (idx < 0 || idx >= cardCount)
    return;

  blockCardIndex = idx;
  currentScreen = SCR_BLOCK_CARD;
  drawBlockCardScreen();
}

void saveEditedCard()
{
  if (editCardIndex < 0 || editCardIndex >= cardCount)
    return;

  cards[editCardIndex].name = editCardName;
  cards[editCardIndex].role = editCardRole;
  saveCardsToSPIFFS();
}

// Save all logs to SPIFFS (backup storage)
void saveLogToSPIFFS()
{
  File file = SPIFFS.open("/access_log.json", "w");
  if (!file)
  {
    Serial.println("[SPIFFS] Failed to open log file for writing");
    return;
  }

  file.print("[");
  int start = (logCount < MAX_LOG) ? 0 : logHead;
  for (int i = 0; i < logCount; i++)
  {
    int idx = (start + i) % MAX_LOG;
    if (i > 0)
      file.print(",");

    String safeName = logBuf[idx].name;
    safeName.replace("\"", "'");

    String safeRole = logBuf[idx].role;
    safeRole.replace("\"", "'");

    String safeTS = logBuf[idx].ts;
    safeTS.replace("\"", "'");
    file.printf("{\"uid\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"ts\":\"%s\",\"granted\":%s}",
                logBuf[idx].uid.c_str(),
                safeName.c_str(),
                safeRole.c_str(),
                safeTS.c_str(),
                logBuf[idx].granted ? "true" : "false");
  }
  file.print("]");
  file.close();
  Serial.printf("[SPIFFS] Saved %d log entries\n", logCount);
}

// Load logs from SPIFFS on startup
void loadLogsFromSPIFFS()
{
  if (!SPIFFS.exists("/access_log.json"))
  {
    Serial.println("[SPIFFS] No backup log found");
    return;
  }

  File file = SPIFFS.open("/access_log.json", "r");
  if (!file)
  {
    Serial.println("[SPIFFS] Failed to open log file");
    return;
  }

  String json = file.readString();
  file.close();

  // Simple JSON parser (basic - just extract field values)
  int pos = 0;
  logCount = 0;
  logHead = 0;

  while (pos < (int)json.length() && logCount < MAX_LOG)
  {
    int uidStart = json.indexOf("\"uid\":\"", pos);
    if (uidStart < 0)
      break;
    uidStart += 8;
    int uidEnd = json.indexOf("\"", uidStart);
    String uid = json.substring(uidStart, uidEnd);

    int nameStart = json.indexOf("\"name\":\"", uidEnd);
    if (nameStart < 0)
      break;
    nameStart += 8;
    int nameEnd = json.indexOf("\"", nameStart);
    String name = json.substring(nameStart, nameEnd);

    int roleStart = json.indexOf("\"role\":\"", nameEnd);
    if (roleStart < 0)
      break;
    roleStart += 8;
    int roleEnd = json.indexOf("\"", roleStart);
    String role = json.substring(roleStart, roleEnd);

    int tsStart = json.indexOf("\"ts\":\"", roleEnd);
    if (tsStart < 0)
      break;
    tsStart += 7;
    int tsEnd = json.indexOf("\"", tsStart);
    String ts = json.substring(tsStart, tsEnd);

    int grantStart = json.indexOf("\"granted\":", tsEnd);
    if (grantStart < 0)
      break;
    grantStart += 10;
    bool granted = (json[grantStart] == 't');

    logBuf[logHead] = {uid, name, role, ts, granted};
    logHead = (logHead + 1) % MAX_LOG;
    logCount++;
    pos = grantStart + 5;
  }

  Serial.printf("[SPIFFS] Loaded %d log entries from backup\n", logCount);
}

void addLog(const String &uid, const String &name,
            const String &role, bool granted)
{
  String ts = getCurrentTimestamp();
  logBuf[logHead] = {uid, name, role, ts, granted};
  logHead = (logHead + 1) % MAX_LOG;
  if (logCount < MAX_LOG)
    logCount++;
}

// Calculate calibration values from captured points
void calculateCalibration()
{
  // Find min/max raw values
  int min_x = calibPoints[0].raw_x, max_x = calibPoints[0].raw_x;
  int min_y = calibPoints[0].raw_y, max_y = calibPoints[0].raw_y;

  for (int i = 1; i < 4; i++)
  {
    min_x = min(min_x, calibPoints[i].raw_x);
    max_x = max(max_x, calibPoints[i].raw_x);
    min_y = min(min_y, calibPoints[i].raw_y);
    max_y = max(max_y, calibPoints[i].raw_y);
  }

  // Calculate touch range (add small margin)
  int range_x = max_x - min_x;
  int range_y = max_y - min_y;

  Serial.println("\n=== CALIBRATION RESULTS ===");
  Serial.printf("X range: %d to %d (range: %d)\n", min_x, max_x, range_x);
  Serial.printf("Y range: %d to %d (range: %d)\n", min_y, max_y, range_y);
  Serial.println("Captured points:");
  for (int i = 0; i < 4; i++)
  {
    Serial.printf("  Point %d: Screen(%d,%d) -> Raw(%d,%d)\n",
                  i, calibPoints[i].screen_x, calibPoints[i].screen_y,
                  calibPoints[i].raw_x, calibPoints[i].raw_y);
  }

  // Print suggested calibration #defines
  Serial.println("\nSuggested calibration values:");
  Serial.printf("#define TOUCH_X_MIN %d\n", min_x);
  Serial.printf("#define TOUCH_X_MAX %d\n", max_x);
  Serial.printf("#define TOUCH_Y_MIN %d\n", min_y);
  Serial.printf("#define TOUCH_Y_MAX %d\n", max_y);
  Serial.println("=== Copy these values to the #define section ===\n");
}

// Map raw touch → screen pixels (portrait mode)
bool getTouchPoint(int &sx, int &sy)
{
  if (!ts.touched())
  {
    touchWasDown = false;
    return false;
  }

  unsigned long now = millis();
  if (now - lastTouch < TOUCH_DEBOUNCE)
    return false;

  long x_sum = 0;
  long y_sum = 0;
  int samples = 0;

  for (int i = 0; i < 3; i++)
  {
    if (!ts.touched())
      break;
    TS_Point p = ts.getPoint();
    x_sum += p.x;
    y_sum += p.y;
    samples++;
    delay(5);
  }

  if (samples == 0)
    return false;

  int p_x = x_sum / samples;
  int p_y = y_sum / samples;
  lastRawTouchX = p_x;
  lastRawTouchY = p_y;
  lastTouch = now;
  touchWasDown = true;

  // Portrait mode: coordinates are inverted, no swap needed
  sx = map(p_x, TOUCH_X_MAX, TOUCH_X_MIN, 0, SCREEN_W);
  sy = map(p_y, TOUCH_Y_MAX, TOUCH_Y_MIN, 0, SCREEN_H);
  sx = constrain(sx, 0, SCREEN_W - 1);
  sy = constrain(sy, 0, SCREEN_H - 1);

  // Uncomment for touch calibration/debug:
  // Serial.printf("TOUCH RAW x=%4d y=%4d -> MAPPED x=%3d y=%3d\n", p_x, p_y, sx, sy);
  return true;
}

// Get raw touch coordinates (for calibration)
bool getRawTouchPoint(int &raw_x, int &raw_y)
{
  if (!ts.touched())
    return false;

  // Average multiple reads for stability
  int x_sum = 0, y_sum = 0;
  for (int i = 0; i < 3; i++)
  {
    if (!ts.touched())
      return false;
    TS_Point p = ts.getPoint();
    x_sum += p.x;
    y_sum += p.y;
  }
  raw_x = x_sum / 3;
  raw_y = y_sum / 3;
  return true;
}

void beepBuzzer(int freq = BUZZER_FREQ, int duration = BUZZER_MS)
{
  tone(BUZZER_PIN, freq, duration);
}

void beepSuccess()
{
  beepBuzzer(2500, 80);
}

void beepFailure()
{
  beepBuzzer(1200, 120);
}

void drawScreenSaver()
{
  // Backlight off — true blackout, not a screensaver
  backlightOff("screensaver");
  tft.fillScreen(C_BG);
  // Note: The screen shows "DO NOT TOUCH" only if display is kept on
  // In true sleep mode, the display can remain in any state since backlight is off
}

void drawCurrentScreen()
{
  switch (currentScreen)
  {
  case SCR_IDLE:
    drawIdle();
    break;
  case SCR_GRANTED:
    // keep last granted screen until it changes
    drawGranted(lastGrantedName, lastGrantedRole);
    break;
  case SCR_DENIED:
    drawDenied(lastDeniedUID);
    break;
  case SCR_BLOCKED:
    drawBlocked(lastBlockedName);
    break;
  case SCR_ADMIN_PIN:
    drawAdminPin();
    break;
  case SCR_ADMIN_MENU:
    drawAdminMenu();
    break;
  case SCR_VIEW_LOG:
    drawLogScreen(logPage);
    break;
  case SCR_SCAN_NEW:
    drawScanNew();
    break;
  case SCR_EDIT_CARD:
    drawEditCardScreen();
    break;
  case SCR_BLOCK_CARD:
    drawBlockCardScreen();
    break;
  case SCR_LOG_WEB:
    drawWebInfo();
    break;
  case SCR_CALIBRATE:
    drawCalibrationScreen();
    break;
  default:
    drawIdle();
    break;
  }
}

// =================================================================
//  DRAW HELPERS
// =================================================================

// ── Common header bar with time ───────────────────────────────
void drawHeader(const char *title, uint16_t color)
{
  // Keep every header the same blue color (C_HEADER) regardless of caller-supplied color.
  tft.fillRect(0, 0, SCREEN_W, 24, C_HEADER);

  int tlen = strlen(title);
  tft.setTextSize(2);
  tft.setTextColor(C_WHITE);
  tft.setCursor(max(4, (SCREEN_W - tlen * 12) / 2), 4);
  tft.print(title);
}

// ── Rounded button with optional press effect ────────────────────
void drawButton(int x, int y, int w, int h,
                const char *label, uint16_t bg, uint16_t fg = C_WHITE, bool pressed = false)
{
  tft.fillRect(x, y, w, h, pressed ? (bg >> 1) : bg);
  tft.drawRect(x, y, w, h, fg);
  int tx = x + (w - strlen(label) * 6) / 2;
  int ty = y + (h - 8) / 2;
  tft.setTextColor(fg);
  tft.setTextSize(1);
  tft.setCursor(tx, ty);
  tft.print(label);
}

void drawInputBox(int x, int y, int w, int h,
                  const char *label, const String &value, bool active)
{
  tft.setTextSize(1);
  tft.setTextColor(active ? C_CYAN : C_GRAY);
  tft.setCursor(x, y - 12);
  tft.print(label);
  tft.fillRect(x, y, w, h, C_PANEL);
  tft.drawRect(x, y, w, h, active ? C_CYAN : C_GRAY);
  tft.setTextColor(C_WHITE);
  tft.setCursor(x + 6, y + (h / 2 - 4));
  String displayText = value;
  if (displayText.length() > 22)
    displayText = displayText.substring(0, 22);
  tft.print(displayText);
}

void drawAdminPinDots()
{
  int centerX = SCREEN_W / 2;
  int dotSpan = 90;
  int dotStartX = centerX - dotSpan / 2;
  tft.fillRect(dotStartX - 10, 58, dotSpan + 20, 24, C_BG);
  for (int i = 0; i < 4; i++)
  {
    int dotX = dotStartX + i * 30;
    int dotY = 70;
    if (i < (int)adminPinEntry.length())
      tft.fillCircle(dotX, dotY, 7, C_GREEN);
    else
      tft.drawCircle(dotX, dotY, 7, C_GRAY);
  }
}

void drawEditCardScreen()
{
  tft.fillScreen(C_BG);
  drawHeader("EDIT CARD", C_PURPLE);

  drawInputBox(8, 44, 224, 30, "Card UID", editCardUid, false);
  drawInputBox(8, 84, 224, 30, "Name", editCardName, editCardField == 0);
  drawInputBox(8, 124, 224, 30, "Role", editCardRole, editCardField == 1);

  bool blockedState = (editCardIndex >= 0 && editCardIndex < cardCount) ? cards[editCardIndex].blocked : false;
  tft.setTextSize(1);
  tft.setTextColor(blockedState ? C_RED : C_GREEN);
  tft.setCursor(8, 164);
  tft.print("Status: ");
  tft.setTextColor(C_WHITE);
  tft.print(blockedState ? "Blocked" : "Active");

  drawButton(8, 184, 224, 28, "Cancel", C_RED, C_WHITE);

  const char *row1 = "QWERTYUIOP";
  const char *row2 = "ASDFGHJKL";
  const char *row3[9] = {"Z", "X", "C", "V", "B", "N", "Space", "<", "OK"};

  int x = 8;
  for (int i = 0; i < 10; i++)
  {
    char label[2] = {row1[i], '\0'};
    drawButton(x, 206, 22, 26, label, C_DKGRAY, C_WHITE);
    x += 24;
  }

  x = 8;
  for (int i = 0; i < 9; i++)
  {
    char label[2] = {row2[i], '\0'};
    drawButton(x, 234, 24, 26, label, C_DKGRAY, C_WHITE);
    x += 26;
  }

  x = 8;
  for (int i = 0; i < 9; i++)
  {
    const char *label = row3[i];
    int w = strcmp(label, "Space") == 0 ? 46 : (strcmp(label, "OK") == 0 ? 26 : 20);
    drawButton(x, 264, w, 26, label, C_DKGRAY, C_WHITE);
    x += w + 2;
  }

  drawNavBar();
}

// ── Fixed Navigation Bar (Bottom) ─────────────────────────────
void drawNavBar()
{
  tft.fillRect(0, NAV_Y, SCREEN_W, NAV_HEIGHT, C_DKGRAY);
  tft.drawLine(0, NAV_Y, SCREEN_W, NAV_Y, C_GRAY);

  drawButton(8, NAV_Y + 2, 110, 24, "< Back", C_RED, C_WHITE);
  drawButton(122, NAV_Y + 2, 110, 24, "Home", C_LBLUE, C_WHITE);
}

// =================================================================
//  SCREEN DRAW FUNCTIONS
// =================================================================

// ── IDLE ──────────────────────────────────────────────────────
void drawIdle()
{
  tft.fillScreen(C_BG);

  // ── Header ─────────────────────────────────────────────────
  tft.fillRect(0, 0, SCREEN_W, 24, C_HEADER);

  // Signal bars — top left
  int heights[] = {5, 9, 13, 18};
  for (int i = 0; i < 4; i++)
  {
    int h = heights[i];
    tft.fillRect(6 + i * 7, 20 - h, 4, h, 0x07E4);
  }

  // Title — centred
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  String l1 = "RND LOCK";
  tft.setCursor((SCREEN_W - l1.length() * 12) / 2, 2);
  tft.print(l1);

  // ── Status bar — WiFi + IP on same line ────────────────────
  tft.fillRect(0, 26, SCREEN_W, 18, 0x0008);
  tft.setTextSize(1);
  if (WiFi.status() == WL_CONNECTED)
  {
    tft.setTextColor(0x07E4); // green
    tft.setCursor(4, 31);
    tft.print("WiFi Connected");
    tft.setTextColor(0x4A5F); // steel blue
    String ip = WiFi.localIP().toString();
    tft.setCursor(SCREEN_W - ip.length() * 6 - 4, 31);
    tft.print(ip);
  }
  else
  {
    tft.setTextColor(C_ORANGE);
    tft.setCursor(4, 31);
    tft.print("Not Connected");
    tft.setTextColor(0x4A5F);
    tft.setCursor(SCREEN_W - 12 * 6 - 4, 31);
    tft.print("0.0.0.0");
  }

  // ── Lock icon ───────────────────────────────────────────────
  int cx = 120, cy = 152;

  // Glow rings
  tft.drawCircle(cx, cy, 52, 0x00A8);
  tft.drawCircle(cx, cy, 46, 0x0149);

  // Shackle
  for (int dx = -1; dx <= 1; dx++)
  {
    tft.drawLine(cx - 18 + dx, cy - 18, cx - 18 + dx, cy - 36, 0x07FF);
    tft.drawLine(cx + 18 + dx, cy - 18, cx + 18 + dx, cy - 36, 0x07FF);
  }
  tft.fillRect(cx - 18, cy - 39, 37, 4, 0x07FF);

  // Lock body
  tft.fillRect(cx - 28, cy - 18, 56, 38, 0x0049);
  tft.drawRect(cx - 28, cy - 18, 56, 38, 0x07FF);
  tft.drawRect(cx - 27, cy - 17, 54, 36, 0x03EF);

  // Keyhole
  tft.fillCircle(cx, cy + 4, 6, 0x07FF);
  tft.fillRect(cx - 3, cy + 9, 6, 9, 0x07FF);

  // ── SCAN TO UNLOCK ──────────────────────────────────────────
  tft.setTextColor(0x07E4);
  tft.setTextSize(1);
  String stl = "SCAN TO UNLOCK";
  tft.setCursor((SCREEN_W - stl.length() * 6) / 2, 216);
  tft.print(stl);

  // Scan bar
  tft.drawRect(70, 227, 100, 3, 0x0149);
  tft.fillRect(70, 227, 40, 3, 0x07E0);

  // ── OR divider ──────────────────────────────────────────────
  tft.drawLine(70, 242, 104, 242, 0x0A3A); // left line
  tft.setTextColor(0x3A6A);
  tft.setCursor(108, 238);
  tft.print("OR");
  tft.drawLine(122, 242, 156, 242, 0x0A3A); // right line

  // ── PIN button ──────────────────────────────────────────────
  drawButton(70, 252, 100, 28, "PIN LOGIN", C_PURPLE, C_WHITE);

  drawNavBar();
}

// ── ACCESS GRANTED ────────────────────────────────────────────
void drawGranted(const String &name, const String &role)
{
  lastGrantedName = name;
  lastGrantedRole = role;
  tft.fillScreen(C_BG);
  drawHeader("ACCESS GRANTED", C_DKGREEN);

  tft.fillRect(12, 46, 216, 160, C_PANEL2);
  tft.drawRect(12, 46, 216, 160, C_GREEN);

  tft.setTextColor(C_GREEN);
  tft.setTextSize(4);
  tft.setCursor(88, 66);
  tft.print("OK");

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 132);
  tft.print("ACCESS GRANTED");

  tft.setTextColor(C_CYAN);
  tft.setTextSize(1);
  tft.setCursor(20, 158);
  tft.print(name);

  tft.setTextColor(C_MUTED);
  tft.setCursor(20, 174);
  tft.print(role);

  tft.setTextColor(C_GRAY);
  tft.setCursor(20, 192);
  tft.print("Door unlocked");
}

// ── ACCESS DENIED ─────────────────────────────────────────────
void drawDenied(const String &uid)
{
  lastDeniedUID = uid;
  tft.fillScreen(C_BG);
  drawHeader("ACCESS DENIED", C_RED);

  tft.fillRect(12, 46, 216, 160, C_PANEL2);
  tft.drawRect(12, 46, 216, 160, C_RED);

  tft.setTextColor(C_RED);
  tft.setTextSize(4);
  tft.setCursor(88, 66);
  tft.print("NO");

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 132);
  tft.print("ACCESS DENIED");

  tft.setTextColor(C_MUTED);
  tft.setTextSize(1);
  tft.setCursor(20, 158);
  tft.print("Unauthorized card");

  tft.setTextColor(C_GRAY);
  tft.setCursor(20, 176);
  tft.print("UID: " + uid);
  tft.setCursor(20, 190);
  tft.print("Please try again.");
}

// ── BLOCKED ───────────────────────────────────────────────────
void drawBlocked(const String &name)
{
  tft.fillScreen(C_BG);
  drawHeader("   CARD BLOCKED ", C_ORANGE >> 1);

  tft.setTextColor(C_ORANGE);
  tft.setTextSize(3);
  tft.setCursor(124, 50);
  tft.print("!");

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 110);
  tft.print("Card blocked:");
  tft.setCursor(8, 135);
  tft.setTextColor(C_ORANGE);
  tft.print(name);

  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(8, 193);
  tft.print("Contact manager.");
}

// ── LOGIN WITH PIN ENTRY ─────────────────────────────
void drawAdminPin()
{
  tft.fillScreen(C_BG);
  drawHeader("PIN LOGIN", C_PURPLE);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  int promptWidth = strlen("Enter your 4-digit PIN") * 6;
  tft.setCursor((SCREEN_W - promptWidth) / 2, 45);
  tft.print("Enter your 4-digit PIN");

  drawAdminPinDots();

  const char *keys[] = {"1", "2", "3",
                        "4", "5", "6",
                        "7", "8", "9",
                        "<x", "0", "OK"};
  uint16_t kcolors[] = {C_DKGRAY, C_DKGRAY, C_DKGRAY,
                        C_DKGRAY, C_DKGRAY, C_DKGRAY,
                        C_DKGRAY, C_DKGRAY, C_DKGRAY,
                        C_RED, C_DKGRAY, C_LBLUE};

  for (int i = 0; i < 12; i++)
  {
    int col = i % 3, row = i / 3;
    int bx = 4 + col * 82;
    int by = 90 + row * 46;
    drawButton(bx, by, 80, 44, keys[i], kcolors[i], C_WHITE);
  }
  drawNavBar();
}

// ── ADMIN MENU ────────────────────────────────────────────────
void drawAdminMenu()
{
  tft.fillScreen(C_BG);
  drawHeader("MANAGEMENT", C_PURPLE);

  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  const char *cardText = "Registered cards: ";
  int cardWidth = strlen(cardText) * 6;
  int cardX = (SCREEN_W - cardWidth - 12) / 2;
  tft.setCursor(cardX, 44);
  tft.print(cardText);
  tft.setTextColor(C_CYAN);
  tft.print(cardCount);

  drawButton(8, 68, 224, 28, "View Log", C_CYAN, C_BG);
  drawButton(8, 100, 224, 28, "Add New Card", C_GREEN, C_BG);
  drawButton(8, 132, 224, 28, "Edit Card", C_ORANGE, C_WHITE);
  drawButton(8, 164, 224, 28, "Block / Unblock", C_RED, C_WHITE);
  drawButton(8, 196, 224, 28, "Web Register", C_LBLUE, C_BG);
  drawButton(8, 228, 224, 28, "Open Lock", C_YELLOW, C_BG);
  drawNavBar();
}

// ── VIEW LOG ──────────────────────────────────────────────────
void drawLogScreen(int page)
{
  tft.fillScreen(C_BG);
  drawHeader("  ACCESS LOG    ", 0x0230);

  int perPage = 3;
  int total = logCount;
  int end = max(0, total - page * perPage);
  int start = max(0, end - perPage);

  tft.setTextSize(1);
  for (int i = start; i < end; i++)
  {
    // Walk backwards from newest: index in circular buffer
    int realIdx = (logHead - 1 - (logCount - 1 - i) % MAX_LOG + MAX_LOG * 2) % MAX_LOG;
    LogEntry &e = logBuf[realIdx];
    int y = 44 + (i - start) * 60;
    uint16_t col = e.granted ? C_DKGREEN : 0x4000;
    tft.fillRect(4, y, 232, 50, col);
    tft.drawRect(4, y, 232, 50, e.granted ? C_GREEN : C_RED);
    tft.setTextColor(C_WHITE);
    tft.setCursor(8, y + 6);
    tft.print(e.name.length() > 18 ? e.name.substring(0, 18) : e.name);
    tft.setTextColor(C_GRAY);
    tft.setCursor(8, y + 22);
    tft.print("Role: " + e.role);
    tft.setCursor(152, y + 6);
    tft.setTextColor(e.granted ? C_GREEN : C_RED);
    tft.print(e.granted ? "GRANTED" : "DENIED");
  }
  drawNavBar();
}

// ── SCAN NEW CARD ─────────────────────────────────────────────
void drawScanNew()
{
  tft.fillScreen(C_BG);
  drawHeader(waitingForEditCard ? "  EDIT CARD  " : "  ADD NEW CARD  ", 0x3000);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 55);
  if (waitingForEditCard)
    tft.print("Scan existing card");
  else
    tft.print("Scan new card");
  tft.setCursor(20, 80);
  tft.print("with RFID reader");

  tft.setTextColor(C_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(120, 112);
  tft.print("...");

  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(20, 172);
  if (waitingForEditCard)
    tft.print("Existing card details can be edited on-screen.");
  else
    tft.print("New card added as 'Staff'. Edit name/role on-screen.");
  drawNavBar();
}

// ── BLOCK / UNBLOCK CARD ───────────────────────────────────────
void drawBlockCardScreen()
{
  tft.fillScreen(C_BG);
  drawHeader("BLOCK CARD", C_RED);

  if (blockCardIndex < 0)
  {
    tft.setTextColor(C_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 55);
    tft.print("Scan existing card");

    tft.setTextSize(1);
    tft.setCursor(20, 90);
    tft.print("to block or unblock on-screen.");

    tft.setTextColor(C_GRAY);
    tft.setCursor(20, 120);
    tft.print("Registered cards can be toggled here.");
  }
  else
  {
    Card &card = cards[blockCardIndex];
    tft.setTextColor(C_WHITE);
    tft.setTextSize(1);
    tft.setCursor(8, 55);
    tft.print("UID:");
    tft.setCursor(8, 70);
    tft.print(card.uid);

    tft.setCursor(8, 94);
    tft.print("Name:");
    tft.setCursor(8, 109);
    tft.print(card.name);

    tft.setCursor(8, 133);
    tft.print("Role:");
    tft.setCursor(8, 148);
    tft.print(card.role);

    tft.setCursor(8, 172);
    tft.setTextColor(card.blocked ? C_RED : C_GREEN);
    tft.print(card.blocked ? "Blocked" : "Active");

    drawButton(8, 196, 224, 28,
               card.blocked ? "UNBLOCK CARD" : "BLOCK CARD",
               card.blocked ? C_DKGREEN : C_RED,
               C_WHITE);
  }

  drawNavBar();
}

// ── WEB REGISTER INFO ─────────────────────────────────────────
void drawWebInfo()
{
  tft.fillScreen(C_BG);
  drawHeader("  WEB REGISTER  ", 0x1082);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(8, 50);
  tft.print("Open on any device on WiFi:");

  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(8, 68);
  if (WiFi.status() == WL_CONNECTED)
    tft.print("http://" + WiFi.localIP().toString());
  else
    tft.print("WiFi not connected");

  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(8, 112);
  tft.print("Features:");
  tft.setCursor(8, 126);
  tft.print("- Full access log with timestamps");
  tft.setCursor(8, 140);
  tft.print("- Add / rename / delete cards");
  tft.setCursor(8, 154);
  tft.print("- Open lock from browser");
  tft.setCursor(8, 168);
  tft.print("- Filter granted / denied entries");

  drawNavBar();
}

// ── CALIBRATION TEST SCREEN ──────────────────────────────────
void drawCalibrationScreen()
{
  tft.fillScreen(C_BG);
  drawHeader("CALIBRATE", C_HEADER);

  tft.setTextColor(C_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(8, 50);
  tft.printf("Point %d/4 - Touch the target:", calibIdx + 1);

  // Draw target circles at each calibration point
  for (int i = 0; i < 4; i++)
  {
    uint16_t color = (i == calibIdx) ? C_GREEN : C_DKGRAY;
    tft.drawCircle(calibPoints[i].screen_x, calibPoints[i].screen_y, 12, color);
    tft.drawCircle(calibPoints[i].screen_x, calibPoints[i].screen_y, 10, color);

    // Fill center of active target
    if (i == calibIdx)
    {
      tft.fillCircle(calibPoints[i].screen_x, calibPoints[i].screen_y, 8, C_GREEN);
    }
    else if (calibPoints[i].captured)
    {
      tft.fillCircle(calibPoints[i].screen_x, calibPoints[i].screen_y, 6, C_DKGREEN);
    }
  }

  // Show coordinates of current target
  tft.setTextColor(C_CYAN);
  tft.setCursor(8, 100);
  tft.printf("Target: (%d, %d)", calibPoints[calibIdx].screen_x, calibPoints[calibIdx].screen_y);

  // Show captured coordinates
  tft.setTextColor(C_GRAY);
  tft.setCursor(8, 130);
  if (calibPoints[calibIdx].captured)
  {
    tft.printf("Raw: (%d, %d)", calibPoints[calibIdx].raw_x, calibPoints[calibIdx].raw_y);
  }
  else
  {
    tft.print("Waiting for touch...");
  }

  // Progress indicator
  tft.setTextColor(C_YELLOW);
  tft.setCursor(8, 160);
  for (int i = 0; i < 4; i++)
  {
    tft.print(calibPoints[i].captured ? "[X] " : "[ ] ");
  }

  // Instructions
  tft.setTextColor(C_ORANGE);
  tft.setTextSize(1);
  tft.setCursor(8, 200);
  tft.print("Press firmly on green circle");
  tft.setCursor(8, 214);
  tft.print("Button: Submit calibration");

  drawButton(8, 250, 100, 28, "Submit", C_GREEN, C_BG);
  drawButton(132, 250, 100, 28, "Cancel", C_RED, C_WHITE);
}

// =================================================================
//  TOUCH HANDLER
// =================================================================
void handleTouch(int tx, int ty)
{
  // Serial.printf("Touch: (%d, %d) screen=%d\n", tx, ty, currentScreen);

  // Global nav-bar handling: Back (left) and Home (right)
  if (ty >= NAV_Y)
  {
    // Back button: x=8..118
    if (tx >= 8 && tx <= 118)
    {
      switch (currentScreen)
      {
      case SCR_VIEW_LOG:
      case SCR_SCAN_NEW:
      case SCR_EDIT_CARD:
      case SCR_LOG_WEB:
        currentScreen = SCR_ADMIN_MENU;
        drawAdminMenu();
        break;
      case SCR_ADMIN_PIN:
      case SCR_ADMIN_MENU:
      case SCR_CALIBRATE:
      default:
        currentScreen = SCR_IDLE;
        drawIdle();
        break;
      }
      return;
    }

    // Home button: x=122..232
    if (tx >= 122 && tx <= 232)
    {
      currentScreen = SCR_IDLE;
      drawIdle();
      return;
    }
  }

  switch (currentScreen)
  {

  // ── CALIBRATION ───────────────────────────────────────────
  case SCR_CALIBRATE:
  {
    // Submit button: x=8..108, y=250..278
    if (tx >= 8 && tx <= 108 && ty >= 250 && ty <= 278)
    {
      // Check if all points captured
      bool allCaptured = true;
      for (int i = 0; i < 4; i++)
      {
        if (!calibPoints[i].captured)
        {
          allCaptured = false;
          break;
        }
      }

      if (allCaptured)
      {
        calculateCalibration();
        currentScreen = SCR_IDLE;
        drawIdle();
      }
      else
      {
        tft.fillRect(0, 180, 240, 20, C_RED);
        tft.setTextColor(C_WHITE);
        tft.setTextSize(1);
        tft.setCursor(20, 185);
        tft.print("Capture all 4 points first!");
        delay(1500);
        drawCalibrationScreen();
      }
      return;
    }

    // Cancel button: x=132..232, y=250..278
    if (tx >= 132 && tx <= 232 && ty >= 250 && ty <= 278)
    {
      // Reset calibration data
      for (int i = 0; i < 4; i++)
        calibPoints[i].captured = false;
      calibIdx = 0;

      currentScreen = SCR_IDLE;
      drawIdle();
      return;
    }

    // Capture touch on target
    if (!calibPoints[calibIdx].captured && tx >= 40 && ty >= 40)
    {
      // Accept touch if near target (within 30 pixels)
      int dx = abs(tx - calibPoints[calibIdx].screen_x);
      int dy = abs(ty - calibPoints[calibIdx].screen_y);

      if (dx <= 30 && dy <= 30)
      {
        // Get raw coordinates for this touch
        int raw_x, raw_y;
        if (getRawTouchPoint(raw_x, raw_y))
        {
          calibPoints[calibIdx].raw_x = raw_x;
          calibPoints[calibIdx].raw_y = raw_y;
          calibPoints[calibIdx].captured = true;

          Serial.printf("Calibration point %d captured: Screen(%d,%d) -> Raw(%d,%d)\n",
                        calibIdx, calibPoints[calibIdx].screen_x, calibPoints[calibIdx].screen_y,
                        raw_x, raw_y);

          // Move to next point
          if (calibIdx < 3)
          {
            calibIdx++;
            delay(300);
          }
          drawCalibrationScreen();
        }
      }
      else
      {
        tft.fillRect(0, 90, 240, 20, C_ORANGE);
        tft.setTextColor(C_BG);
        tft.setTextSize(1);
        tft.setCursor(20, 95);
        tft.print("Touch the green circle exactly!");
        delay(800);
        drawCalibrationScreen();
      }
    }
    break;
  }

  // ── PIN ENTRY ─────────────────────────────────────────────
  case SCR_ADMIN_PIN:
  {
    // Numpad grid: 3 cols × 4 rows (matches drawAdminPin layout)
    if (tx >= 4 && tx <= 248 && ty >= 90 && ty < 280)
    {
      int col = (tx - 4) / 82;  // horiz step used in draw
      int row = (ty - 90) / 46; // vertical step used in draw
      int key = row * 3 + col;
      const char *keys[] = {"1", "2", "3",
                            "4", "5", "6",
                            "7", "8", "9",
                            "<x", "0", "OK"};
      if (key >= 0 && key < 12)
      {
        String k = keys[key];
        if (k == "<x")
        {
          if (adminPinEntry.length())
            adminPinEntry.remove(adminPinEntry.length() - 1);
        }
        else if (k == "OK")
        {
          if (adminPinEntry == ADMIN_PIN)
          {
            beepSuccess();
            currentScreen = SCR_ADMIN_MENU;
            drawAdminMenu();
          }
          else
          {
            beepFailure();
            // Non-blocking error feedback
            showingMessage = true;
            messageText = "Wrong PIN! Try again.";
            messageTimeout = millis() + 1000;
            adminPinEntry = "";
            // Flash error on screen
            tft.fillRect(30, 74, 180, 18, C_RED);
            tft.setTextColor(C_WHITE);
            tft.setTextSize(1);
            tft.setCursor(55, 76);
            tft.print(messageText);
          }
          return;
        }
        else if (adminPinEntry.length() < 4)
        {
          adminPinEntry += k;
        }
        drawAdminPinDots();
      }
    }
    break;
  }

  // ── ADMIN MENU ────────────────────────────────────────────
  case SCR_ADMIN_MENU:
    if (tx > 8 && tx < 232 && ty > 58 && ty < NAV_Y)
    {
      if (ty >= 60 && ty <= 100)
      { // View Log — button drawn at y=68, h=28 → covers 68-96, allow ±8px
        logPage = 0;
        currentScreen = SCR_VIEW_LOG;
        drawLogScreen(0);
      }
      else if (ty >= 100 && ty <= 134)
      { // Add New Card — button drawn at y=100, h=28 → covers 100-128
        waitingForNewCard = true;
        waitingForEditCard = false;
        editExistingMode = false;
        currentScreen = SCR_SCAN_NEW;
        drawScanNew();
      }
      else if (ty >= 134 && ty <= 168)
      { // Edit Card — button drawn at y=132, h=28 → covers 132-160
        waitingForEditCard = true;
        waitingForNewCard = false;
        waitingForBlockCard = false;
        editExistingMode = true;
        currentScreen = SCR_SCAN_NEW;
        drawScanNew();
      }
      else if (ty >= 168 && ty <= 202)
      { // Block / Unblock — button drawn at y=164, h=28 → covers 164-192
        waitingForBlockCard = true;
        waitingForNewCard = false;
        waitingForEditCard = false;
        blockCardIndex = -1;
        currentScreen = SCR_BLOCK_CARD;
        drawBlockCardScreen();
      }
      else if (ty >= 202 && ty <= 236)
      { // Web Register — button drawn at y=196, h=28 → covers 196-224
        currentScreen = SCR_LOG_WEB;
        drawWebInfo();
      }
      else if (ty >= 236 && ty <= 270)
      { // Open Lock — button drawn at y=228, h=28 → covers 228-256
        if (!doorUnlocked)
        {
          tft.fillRect(8, 228, 224, 28, C_YELLOW);
          tft.setTextColor(C_BG);
          tft.setTextSize(1);
          tft.setCursor(85, 238);
          tft.print("Opening...");

          unlockDoor("admin manual"); // ← centralised call
          doorUnlocked = true;
          unlockUntil = millis() + UNLOCK_MS;

          messageTimeout = millis() + UNLOCK_MS;
          showingMessage = true;
          messageText = "Door unlocking...";

          // Return to idle; RFID scans are blocked while the door is unlocked
          currentScreen = SCR_IDLE;
        }
        else
        {
          showingMessage = true;
          messageText = "Already unlocked";
          messageTimeout = millis() + MSG_MS;
        }
      }
    }
    break;

  // ── VIEW LOG ──────────────────────────────────────────────
  case SCR_VIEW_LOG:
    // Page controls: tap area just above nav bar (left=Newer, right=Older)
    if (ty >= NAV_Y - 36 && ty < NAV_Y)
    {
      if (tx >= 8 && tx <= 116)
      { // Newer
        if (logPage > 0)
        {
          logPage--;
          drawLogScreen(logPage);
        }
      }
      else if (tx >= 124 && tx <= 232)
      { // Older
        logPage++;
        drawLogScreen(logPage);
      }
    }
    break;

  // ── SCAN NEW CARD — Cancel ────────────────────────────────
  case SCR_SCAN_NEW:
    // Nav bar handles cancel/back
    break;

  // ── EDIT CARD SCREEN ──────────────────────────────────────
  case SCR_EDIT_CARD:
  {
    // Cancel button
    if (tx >= 8 && tx <= 232 && ty >= 184 && ty <= 212)
    {
      if (!editExistingMode && editCardIndex == cardCount - 1 && cardCount > 0)
      {
        cardCount--;
        saveCardsToSPIFFS();
      }
      currentScreen = SCR_ADMIN_MENU;
      drawAdminMenu();
      return;
    }

    // Select name field
    if (tx >= 8 && tx <= 232 && ty >= 84 && ty <= 114)
    {
      editCardField = 0;
      drawEditCardScreen();
      return;
    }

    // Select role field
    if (tx >= 8 && tx <= 232 && ty >= 124 && ty <= 154)
    {
      editCardField = 1;
      drawEditCardScreen();
      return;
    }

    // Keyboard row 1
    if (ty >= 206 && ty < 232)
    {
      int keyX = 8;
      for (int i = 0; i < 10; i++)
      {
        if (tx >= keyX && tx < keyX + 22)
        {
          char key[2] = {(char)"QWERTYUIOP"[i], '\0'};
          if (editCardField == 0 && editCardName.length() < 24)
            editCardName += key;
          else if (editCardField == 1 && editCardRole.length() < 24)
            editCardRole += key;
          drawEditCardScreen();
          return;
        }
        keyX += 24;
      }
    }

    // Keyboard row 2
    if (ty >= 234 && ty < 260)
    {
      int keyX = 8;
      for (int i = 0; i < 9; i++)
      {
        if (tx >= keyX && tx < keyX + 24)
        {
          char key[2] = {(char)"ASDFGHJKL"[i], '\0'};
          if (editCardField == 0 && editCardName.length() < 24)
            editCardName += key;
          else if (editCardField == 1 && editCardRole.length() < 24)
            editCardRole += key;
          drawEditCardScreen();
          return;
        }
        keyX += 26;
      }
    }

    // Keyboard row 3 (with special keys)
    if (ty >= 264 && ty < 290)
    {
      int keyX = 8;
      const char *row3[9] = {"Z", "X", "C", "V", "B", "N", "Space", "<", "OK"};
      for (int i = 0; i < 9; i++)
      {
        int keyW = strcmp(row3[i], "Space") == 0 ? 46 : (strcmp(row3[i], "OK") == 0 ? 26 : 20);
        if (tx >= keyX && tx < keyX + keyW)
        {
          if (strcmp(row3[i], "Space") == 0)
          {
            if (editCardField == 0 && editCardName.length() < 24)
              editCardName += ' ';
            else if (editCardField == 1 && editCardRole.length() < 24)
              editCardRole += ' ';
          }
          else if (strcmp(row3[i], "<") == 0)
          {
            if (editCardField == 0 && editCardName.length())
              editCardName.remove(editCardName.length() - 1);
            else if (editCardField == 1 && editCardRole.length())
              editCardRole.remove(editCardRole.length() - 1);
          }
          else if (strcmp(row3[i], "OK") == 0)
          {
            saveEditedCard();
            currentScreen = SCR_ADMIN_MENU;
            drawAdminMenu();
            return;
          }
          else
          {
            char key[2] = {(char)row3[i][0], '\0'};
            if (editCardField == 0 && editCardName.length() < 24)
              editCardName += key;
            else if (editCardField == 1 && editCardRole.length() < 24)
              editCardRole += key;
          }
          drawEditCardScreen();
          return;
        }
        keyX += keyW + 2;
      }
    }
    break;
  }

  case SCR_BLOCK_CARD:
    if (blockCardIndex >= 0 && tx >= 8 && tx <= 232 && ty >= 196 && ty <= 224)
    {
      Card &card = cards[blockCardIndex];
      card.blocked = !card.blocked;
      saveCardsToSPIFFS();
      showingMessage = true;
      messageText = card.blocked ? "Card blocked" : "Card active";
      messageTimeout = millis() + 1200;
      drawBlockCardScreen();
      return;
    }
    break;

  // ── WEB INFO — Back ───────────────────────────────────────
  case SCR_LOG_WEB:
    // Nav bar handles back
    break;
    break;

  // ── IDLE ──────────────────────────────────────────────────
  case SCR_IDLE:
    if (tx >= 70 && tx <= 170 && ty >= 256 && ty <= 286)
    {
      adminPinEntry = "";
      currentScreen = SCR_ADMIN_PIN;
      drawAdminPin();
      return;
    }
    // CALIBRATION MODE: Tap header 3 times rapidly to enter calibration
    else if (ty < 40 && tx > 80 && tx < 160)
    {
      static unsigned long lastHeaderTap = 0;
      static int headerTaps = 0;
      unsigned long now = millis();

      if (now - lastHeaderTap < 600)
      {
        headerTaps++;
        if (headerTaps >= 3)
        {
          headerTaps = 0;
          // Enter calibration mode
          calibIdx = 0;
          for (int i = 0; i < 4; i++)
            calibPoints[i].captured = false;
          currentScreen = SCR_CALIBRATE;
          drawCalibrationScreen();
          return;
        }
      }
      else
      {
        headerTaps = 1;
      }
      lastHeaderTap = now;
    }
    break;

  default:
    break;
  }
}

// =================================================================
//  WEB SERVER — HTML dashboard + JSON API
// =================================================================
void handleRoot()
{
  String html = R"RAW(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Door Register</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--bd:#30363d;--green:#3fb950;
  --red:#f85149;--blue:#58a6ff;--muted:#8b949e;--text:#e6edf3}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;margin:0 auto;max-width:1200px;}
.hd{background:var(--card);border-bottom:1px solid var(--bd);
    padding:14px 20px;display:flex;align-items:center;gap:10px}
.hd h1{font-size:16px;font-weight:600}
.dot{width:7px;height:7px;border-radius:50%;background:var(--green);animation:pulse 2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.25}}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;padding:16px 20px}
.stat{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:14px}
.sn{font-size:26px;font-weight:700}.sl{font-size:11px;color:var(--muted);margin-top:3px}
.wrap{background:var(--card);border:1px solid var(--bd);border-radius:18px;padding:0 20px 20px}
.layout{display:grid;gap:16px;margin:0 20px 20px}
@media(min-width:900px){.layout{grid-template-columns:1.45fr 1fr}}
.bar{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap;align-items:center}
.btn{background:#21262d;border:1px solid var(--bd);color:var(--text);
     border-radius:6px;padding:8px 12px;font-size:12px;cursor:pointer}
.btn.on{border-color:var(--blue);color:var(--blue)}
.src{background:#21262d;border:1px solid var(--bd);color:var(--text);
     border-radius:6px;padding:8px 12px;font-size:12px;outline:none}
.src.small{width:120px}
@media(max-width:760px){
  .stats{grid-template-columns:1fr;}
  .layout{margin:0 12px 20px;}
  .bar{flex-direction:column;align-items:stretch;}
  .bar .btn,.bar .src{width:100%;max-width:100%;}
  .wrap{border-radius:14px;padding:14px;}
  .card-actions{justify-content:flex-start;}
  .uid{word-break:break-all;}
}
@media(max-width:520px){
  .hd{flex-direction:column;align-items:flex-start;gap:8px;}
  .stats{gap:8px;}
  .bar{gap:6px;}
  table, th, td{font-size:11px;}
}
table{width:100%;border-collapse:collapse;font-size:12px}
th{text-align:left;padding:7px 10px;border-bottom:1px solid var(--bd);
   color:var(--muted);font-weight:500;font-size:11px;text-transform:uppercase}
td{padding:9px 10px;border-bottom:1px solid #21262d}
tr:hover td{background:#1c2128}
.g{background:#1a3c28;color:var(--green);border-radius:10px;padding:2px 9px;font-size:11px;font-weight:600}
.d{background:#3c1a1a;color:var(--red);border-radius:10px;padding:2px 9px;font-size:11px;font-weight:600}
.role{background:#21262d;color:var(--blue);border-radius:10px;padding:2px 8px;font-size:11px}
.uid{font-family:monospace;color:var(--muted);font-size:11px}
.card-actions{display:flex;gap:6px;flex-wrap:wrap}
.card-actions .btn{padding:4px 8px;font-size:11px}
</style></head><body>
<div class="hd"><div class="dot"></div><h1>Smart Door Register</h1>
<span style="margin-left:auto;font-size:11px;color:var(--muted)" id="upd">Live</span></div>
<div class="stats">
<div class="stat"><div class="sn" id="s1">-</div><div class="sl">Total scans</div></div>
<div class="stat"><div class="sn" style="color:var(--green)" id="s2">-</div><div class="sl">Granted</div></div>
<div class="stat"><div class="sn" style="color:var(--red)" id="s3">-</div><div class="sl">Denied</div></div>
</div>
<div class="layout">
<div class="wrap">
<div class="bar">
<button class="btn log-btn on" onclick="filt('all',this)">All</button>
<button class="btn log-btn" onclick="filt('granted',this)">Granted</button>
<button class="btn log-btn" onclick="filt('denied',this)">Denied</button>
<button class="btn" onclick="clearLogs()">Clear Logs</button>
<input class="src small" id="openPassword" type="password" placeholder="Web password" style="width:160px">
<button class="btn" onclick="openDoor()">Open Lock</button>
<input class="src" id="q" placeholder="Search name..." style="margin-left:auto;width:180px" oninput="render()">
</div>
<div style="margin-bottom:12px;color:var(--muted);font-size:12px;display:flex;gap:12px;flex-wrap:wrap;align-items:center">
<div id="doorStatus">Door ready</div>
<div id="cardStatus"></div>
</div>
<table><thead><tr><th>#</th><th>Name</th><th>UID</th><th>Role</th><th>Status</th></tr></thead>
<tbody id="tb"><tr><td colspan="5" style="text-align:center;padding:20px;color:var(--muted)">Loading...</td></tr></tbody>
</table>
</div>
<div class="wrap" style="padding-top:0">
<div style="padding:14px 0 6px 0;color:var(--text);font-size:14px;font-weight:600">Card Manager</div>
<div class="bar">
<button class="btn card-btn on" onclick="cardFilter('all',this)">All</button>
<button class="btn card-btn" onclick="cardFilter('active',this)">Active</button>
<button class="btn card-btn" onclick="cardFilter('blocked',this)">Blocked</button>
<input class="src" id="cardQ" placeholder="Search cards..." style="margin-left:auto;width:180px" oninput="renderCards()">
</div>
<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:10px">
<input class="src" id="uid" placeholder="UID">
<input class="src" id="name" placeholder="Name">
<input class="src" id="role" placeholder="Role">
</div>
<input type="hidden" id="originalUid">
<div style="display:flex;gap:10px;align-items:center;margin-bottom:14px;flex-wrap:wrap">
<label style="color:var(--muted);font-size:12px"><input type="checkbox" id="blocked" style="margin-right:6px">Blocked</label>
<button class="btn on" onclick="saveCard()">Save</button>
<button class="btn" onclick="resetCardForm()">Clear</button>
<span id="cardMsg" style="margin-left:auto;color:var(--green);font-size:12px"></span>
</div>
<table><thead><tr><th>#</th><th>Name</th><th>UID</th><th>Role</th><th>Status</th><th>Actions</th></tr></thead>
<tbody id="cardTb"><tr><td colspan="6" style="text-align:center;padding:20px;color:var(--muted)">Loading...</td></tr></tbody>
</table>
</div>
<div class="wrap" style="padding-top:0">
<div style="padding:14px 0 6px 0;color:var(--text);font-size:14px;font-weight:600">Firmware Update</div>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" style="margin-bottom:10px;">
<button class="btn" type="submit">Upload Firmware</button>
</form>
</div>
</div>
<script>
let all=[],cur='all';
let allCards=[];
let cardFilterMode='all';
let editing=false;
async function load(){
  await Promise.all([loadLogs(), loadCards()]);
}
async function loadLogs(){
  try{
    const r=await fetch('/api/log');
    all=await r.json();
    const g=all.filter(e=>e.granted).length;
    document.getElementById('s1').textContent=all.length;
    document.getElementById('s2').textContent=g;
    document.getElementById('s3').textContent=all.length-g;
    document.getElementById('upd').textContent='Updated '+new Date().toLocaleTimeString();
    render();
  }catch(e){
    document.getElementById('tb').innerHTML='<tr><td colspan="5" style="text-align:center;color:var(--red);padding:20px">Connection error</td></tr>';
  }
}
async function loadCards(){
  try{
    const r=await fetch('/api/cards');
    allCards=await r.json();
    document.getElementById('cardStatus').textContent = allCards.length + ' cards loaded';
    renderCards();
  }catch(e){
    document.getElementById('cardTb').innerHTML='<tr><td colspan="6" style="text-align:center;color:var(--red);padding:20px">Connection error</td></tr>';
  }
}
function filt(f,b){cur=f;document.querySelectorAll('.log-btn').forEach(x=>x.classList.remove('on'));b.classList.add('on');render();}
function cardFilter(f,b){cardFilterMode=f;document.querySelectorAll('.card-btn').forEach(x=>x.classList.remove('on'));b.classList.add('on');renderCards();}
function render(){
  const q=document.getElementById('q').value.toLowerCase();
  const d=all.filter(e=>(cur==='all'||(cur==='granted'&&e.granted)||(cur==='denied'&&!e.granted))&&(!q||e.name.toLowerCase().includes(q))).slice().reverse();
  document.getElementById('tb').innerHTML=d.map((e,i)=>`<tr>
    <td style="color:var(--muted)">${d.length-i}</td>
    <td style="font-weight:500">${e.name}</td>
    <td class="uid">${e.uid}</td>
    <td><span class="role">${e.role}</span></td>
    <td><span class="${e.granted?'g':'d'}">${e.granted?'&#10003; Granted':'&#10007; Denied'}</span></td>
  </tr>`).join('')||`<tr><td colspan="5" style="text-align:center;color:var(--muted);padding:20px">No records</td></tr>`;
}
function renderCards(){
  const q=document.getElementById('cardQ').value.toLowerCase();
  const filtered = allCards.filter(c=>{
    if (cardFilterMode==='active' && c.blocked) return false;
    if (cardFilterMode==='blocked' && !c.blocked) return false;
    if (!q) return true;
    return c.name.toLowerCase().includes(q) || c.uid.toLowerCase().includes(q) || c.role.toLowerCase().includes(q);
  });
  document.getElementById('cardTb').innerHTML = filtered.map((c,i)=>`<tr>
    <td style="color:var(--muted)">${i+1}</td>
    <td style="font-weight:500">${c.name}</td>
    <td class="uid">${c.uid}</td>
    <td><span class="role">${c.role}</span></td>
    <td><span class="${c.blocked?'d':'g'}">${c.blocked?'Blocked':'Active'}</span></td>
    <td class="card-actions"><button class="btn" onclick="editCard('${c.uid}')">Edit</button><button class="btn" onclick="deleteCard('${c.uid}')">Delete</button></td>
  </tr>`).join('')||`<tr><td colspan="6" style="text-align:center;color:var(--muted);padding:20px">No cards</td></tr>`;
}
async function sendCardAction(action,payload){
  payload.action = action;
  const r = await fetch('/api/cards', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(payload)
  });
  return await r.json();
}
async function saveCard(){
  const uid = document.getElementById('uid').value.trim();
  const name = document.getElementById('name').value.trim();
  const role = document.getElementById('role').value.trim();
  const blocked = document.getElementById('blocked').checked;
  const oldUid = document.getElementById('originalUid').value.trim();
  if (!uid || !name)
  {
    document.getElementById('cardMsg').textContent = 'UID and name required';
    return;
  }
  const action = editing ? 'update' : 'add';
  const payload = {uid, name, role, blocked};
  if (editing)
    payload.oldUid = oldUid;
  const res = await sendCardAction(action, payload);
  document.getElementById('cardMsg').textContent = res.success ? res.message : res.message || 'Failed';
  if (res.success) {
    resetCardForm();
    await loadCards();
  }
}
function editCard(uid){
  const card = allCards.find(c=>c.uid===uid);
  if (!card) return;
  editing = true;
  document.getElementById('uid').value = card.uid;
  document.getElementById('originalUid').value = card.uid;
  document.getElementById('name').value = card.name;
  document.getElementById('role').value = card.role;
  document.getElementById('blocked').checked = card.blocked;
  document.getElementById('cardMsg').textContent = 'Edit mode';
}
async function deleteCard(uid){
  if (!confirm('Delete card ' + uid + '?')) return;
  const res = await sendCardAction('delete', {uid});
  document.getElementById('cardMsg').textContent = res.success ? res.message : res.message || 'Delete failed';
  if (res.success) await loadCards();
}
function resetCardForm(){
  editing = false;
  document.getElementById('uid').value = '';
  document.getElementById('originalUid').value = '';
  document.getElementById('name').value = '';
  document.getElementById('role').value = '';
  document.getElementById('blocked').checked = false;
  document.getElementById('cardMsg').textContent = '';
}
async function openDoor(){
  const password = document.getElementById('openPassword').value.trim();
  document.getElementById('doorStatus').textContent = 'Opening door...';
  try{
    const r=await fetch('/api/open', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({password})
    });
    const data=await r.json();
    document.getElementById('doorStatus').textContent = data.message || (data.success ? 'Door opened' : 'Open failed');
    if (!data.success) document.getElementById('doorStatus').style.color = 'var(--red)';
    else document.getElementById('doorStatus').style.color = 'var(--green)';
    if (data.success) loadLogs();
    setTimeout(()=>{
      document.getElementById('doorStatus').textContent='Door ready';
      document.getElementById('doorStatus').style.color='var(--text)';
    }, 3000);
  }catch(e){
    document.getElementById('doorStatus').textContent = 'Connection failed';
    document.getElementById('doorStatus').style.color='var(--red)';
  }
}
async function clearLogs(){
  if (!confirm('Clear all access logs?')) return;
  document.getElementById('doorStatus').textContent = 'Clearing logs...';
  try{
    const r=await fetch('/api/clearlogs', {method:'POST'});
    const data=await r.json();
    document.getElementById('doorStatus').textContent = data.message || (data.success ? 'Logs cleared' : 'Clear failed');
    document.getElementById('doorStatus').style.color = data.success ? 'var(--green)' : 'var(--red)';
    if (data.success) {
      all=[];
      render();
      document.getElementById('s1').textContent='0';
      document.getElementById('s2').textContent='0';
      document.getElementById('s3').textContent='0';
    }
    setTimeout(()=>{
      document.getElementById('doorStatus').textContent='Door ready';
      document.getElementById('doorStatus').style.color='var(--text)';
    }, 3000);
  }catch(e){
    document.getElementById('doorStatus').textContent = 'Connection failed';
    document.getElementById('doorStatus').style.color='var(--red)';
  }
}
load();setInterval(load,5000);
</script></body></html>)RAW";
  server.send(200, "text/html", html);
}

void handleLogJSON()
{
  String json = "[";
  int start = (logCount < MAX_LOG) ? 0 : logHead;
  for (int i = 0; i < logCount; i++)
  {
    int idx = (start + i) % MAX_LOG;
    if (i)
      json += ",";
    // Escape name in case it contains quotes
    String safeName = logBuf[idx].name;
    safeName.replace("\"", "'");
    json += "{\"uid\":\"" + logBuf[idx].uid + "\","
                                              "\"name\":\"" +
            safeName + "\","
                       "\"role\":\"" +
            logBuf[idx].role + "\","
                               "\"granted\":" +
            (logBuf[idx].granted ? "true" : "false") + ","
                                                       "\"ts\":\"" +
            logBuf[idx].ts + "\"}";
  }
  json += "]";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleClearLogs()
{
  logCount = 0;
  logHead = 0;
  saveLogToSPIFFS();
  Serial.println("[WEB] Log history cleared via web app");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Logs cleared\"}");
}

String escapeJsonString(const String &text)
{
  String out = text;
  out.replace("\\", "\\\\");
  out.replace("\"", "\\\"");
  return out;
}

String jsonValue(const String &json, const String &key)
{
  String token = "\"" + key + "\":";
  int pos = json.indexOf(token);
  if (pos < 0)
    return "";
  pos += token.length();
  while (pos < json.length())
  {
    char c = json.charAt(pos);
    if (c != ' ' && c != '\n' && c != '\r' && c != '\t')
      break;
    pos++;
  }
  if (pos >= json.length())
    return "";
  if (json.charAt(pos) == '"')
  {
    pos++;
    int end = json.indexOf('"', pos);
    if (end < 0)
      return "";
    return json.substring(pos, end);
  }
  int end = pos;
  while (end < json.length() && json.charAt(end) != ',' && json.charAt(end) != '}' && json.charAt(end) != ']')
    end++;
  return json.substring(pos, end);
}

bool jsonBool(const String &json, const String &key)
{
  String value = jsonValue(json, key);
  return value.equalsIgnoreCase("true");
}

void saveCardsToSPIFFS()
{
  File file = SPIFFS.open("/cards.json", "w");
  if (!file)
  {
    Serial.println("[SPIFFS] Failed to open cards file for writing");
    return;
  }

  file.print("[");
  for (int i = 0; i < cardCount; i++)
  {
    if (i > 0)
      file.print(",");
    file.printf(
        "{\"uid\":\"%s\",\"name\":\"%s\",\"role\":\"%s\",\"blocked\":%s}",
        escapeJsonString(cards[i].uid).c_str(),
        escapeJsonString(cards[i].name).c_str(),
        escapeJsonString(cards[i].role).c_str(),
        cards[i].blocked ? "true" : "false");
  }
  file.print("]");
  file.close();
  Serial.printf("[SPIFFS] Saved %d cards\n", cardCount);
}

void loadCardsFromSPIFFS()
{
  if (!SPIFFS.exists("/cards.json"))
  {
    Serial.println("[SPIFFS] No card database found; starting with empty card list");
    cardCount = 0;
    return;
  }

  File file = SPIFFS.open("/cards.json", "r");
  if (!file)
  {
    Serial.println("[SPIFFS] Failed to open cards file");
    return;
  }

  String json = file.readString();
  file.close();
  if (json.length() == 0)
  {
    Serial.println("[SPIFFS] Cards file empty; starting with empty card list");
    cardCount = 0;
    return;
  }

  int pos = 0;
  int loaded = 0;
  while (true)
  {
    int start = json.indexOf('{', pos);
    if (start < 0 || loaded >= MAX_CARDS)
      break;
    int end = json.indexOf('}', start);
    if (end < 0)
      break;
    String item = json.substring(start, end + 1);
    String uid = jsonValue(item, "uid");
    if (uid.length() > 0)
    {
      cards[loaded].uid = uid;
      cards[loaded].name = jsonValue(item, "name");
      cards[loaded].role = jsonValue(item, "role");
      cards[loaded].blocked = jsonBool(item, "blocked");
      loaded++;
    }
    pos = end + 1;
  }

  if (loaded > 0)
  {
    cardCount = loaded;
    Serial.printf("[SPIFFS] Loaded %d cards\n", cardCount);
  }
  else
  {
    cardCount = 0;
    Serial.println("[SPIFFS] No cards loaded from file; card list is empty");
  }
}

void handleCardsJSON()
{
  String json = "[";
  for (int i = 0; i < cardCount; i++)
  {
    if (i > 0)
      json += ",";
    String safeName = cards[i].name;
    safeName.replace('"', '\'');
    String safeRole = cards[i].role;
    safeRole.replace('"', '\'');
    json += "{\"uid\":\"" + cards[i].uid + "\","
                                           "\"name\":\"" +
            safeName + "\","
                       "\"role\":\"" +
            safeRole + "\","
                       "\"blocked\":" +
            (cards[i].blocked ? "true" : "false") + "}";
  }
  json += "]";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleCardModify()
{
  String body = server.arg("plain");
  if (body.length() == 0)
  {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Empty request body\"}");
    return;
  }

  String action = jsonValue(body, "action");
  String uid = jsonValue(body, "uid");
  String oldUid = jsonValue(body, "oldUid");
  String name = jsonValue(body, "name");
  String role = jsonValue(body, "role");
  bool blocked = jsonBool(body, "blocked");

  if (action == "add")
  {
    if (uid.length() == 0 || name.length() == 0)
    {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"UID and name are required\"}");
      return;
    }
    if (findCard(uid) >= 0)
    {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"Card already exists\"}");
      return;
    }
    if (cardCount >= MAX_CARDS)
    {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"Card limit reached\"}");
      return;
    }
    cards[cardCount++] = {uid, name, role, blocked};
    saveCardsToSPIFFS();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Card added\"}");
    return;
  }

  if (action == "update")
  {
    if (oldUid.length() == 0)
      oldUid = uid;

    int idx = findCard(oldUid);
    if (idx < 0)
    {
      server.send(404, "application/json", "{\"success\":false,\"message\":\"Card not found\"}");
      return;
    }

    if (uid.length() == 0 || name.length() == 0)
    {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"UID and name are required\"}");
      return;
    }

    if (!uid.equalsIgnoreCase(oldUid) && findCard(uid) >= 0)
    {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"New UID already in use\"}");
      return;
    }

    cards[idx].uid = uid;
    cards[idx].name = name;
    cards[idx].role = role;
    cards[idx].blocked = blocked;
    saveCardsToSPIFFS();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Card updated\"}");
    return;
  }

  if (action == "delete")
  {
    if (uid.length() == 0)
      uid = oldUid;

    int idx = findCard(uid);
    if (idx < 0)
    {
      server.send(404, "application/json", "{\"success\":false,\"message\":\"Card not found\"}");
      return;
    }

    for (int i = idx; i < cardCount - 1; i++)
      cards[i] = cards[i + 1];
    cardCount--;
    saveCardsToSPIFFS();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Card deleted\"}");
    return;
  }

  server.send(400, "application/json", "{\"success\":false,\"message\":\"Unknown action\"}");
}

void handleOpenDoor()
{
  String body = server.arg("plain");
  String password = jsonValue(body, "password");
  Serial.println("[WEB] Open door requested");
  if (password.length() == 0 || password != WEB_PASSWORD)
  {
    Serial.println("[WEB] Invalid password for web door open");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(401, "application/json", "{\"success\":false,\"message\":\"Invalid password\"}");
    return;
  }

  if (doorUnlocked)
  {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(409, "application/json", "{\"success\":false,\"message\":\"Door already unlocked\"}");
    return;
  }

  unlockDoor("web manual");
  addLog("WEB", "Web Door", "Admin", true);
  saveLogToSPIFFS();
  drawGranted("Web Access", "Remote unlock");
  doorUnlocked = true;
  unlockUntil = millis() + UNLOCK_MS;
  showingMessage = true;
  messageText = "Web unlock";
  messageTimeout = millis() + MSG_MS;
  currentScreen = SCR_IDLE;
  backlightOn("web dashboard unlock"); // Wake screen as side effect of web unlock
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Door opened\"}");
}

// =================================================================
//  SETUP
// =================================================================
void setup()
{
  Serial.begin(115200);
  delay(200);

  // ── SPIFFS (persistent storage for logs) ────────────────────
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS mount failed!");
  }
  else
  {
    Serial.println("SPIFFS initialized — loading backup logs...");
    loadLogsFromSPIFFS();
    loadCardsFromSPIFFS();
  }

  // ── Relay — initialise LOCKED, then run diagnostic ───────────
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOCK_CLOSE); // Ensure locked at boot
  relayDiagnostic();                   // ← pulse test: listen for two clicks

  // TFT (HSPI)
  hSPI.begin(TFT_SCK, TOUCH_MISO, TFT_MOSI, TFT_CS);
  hSPI.setFrequency(20000000); // 20MHz for faster display updates
  tft.begin();
  tft.setRotation(0); // Portrait mode
  tft.fillScreen(C_BG);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 90);
  tft.print("System is");
  tft.setTextSize(1);
  tft.setTextColor(C_GRAY);
  tft.setCursor(20, 116);
  tft.print("Initializing...");

  // Touch (shares HSPI)
  ts.begin(hSPI);
  ts.setRotation(0); // Portrait mode
  pinMode(TOUCH_IRQ, INPUT_PULLUP);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Exit button
  pinMode(EXIT_BUTTON_PIN, INPUT_PULLUP);

  // ── Backlight PWM setup (LEDC) ─────────────────────────────
  // Configure LEDC for PWM control of backlight (using new ESP32 Arduino core 3.0+ API)
  ledcAttach(TFT_LED_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  backlightOn("startup"); // Start with backlight ON

  // Optional: Outside unlock button (second touch button)
  // pinMode(OUTSIDE_TOUCH_PIN, INPUT_PULLUP);  // Uncomment when wiring is complete

  // RFID (VSPI) — init
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS); // VSPI
  tft.setTextColor(C_YELLOW);
  tft.setCursor(20, 136);
  tft.print("Initializing RFID...");

  bool rfidOK = false;
  for (int i = 0; i < 5; i++)
  {
    rfid.PCD_Init();
    delay(100);
    // Check if PCD version is valid (0x91-0x92 = good)
    byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
    if (v == 0x91 || v == 0x92)
    {
      rfidOK = true;
      Serial.println("RFID version OK: 0x" + String(v, HEX));
      rfid.PCD_DumpVersionToSerial();
      break;
    }
    Serial.println("RFID init retry... (v=" + String(v, HEX) + ")");
  }

  if (!rfidOK)
  {
    Serial.println("WARNING: RFID failed to initialize!");
  }
  else
  {
    Serial.println("RFID ready.");
  }

  // WiFi
  tft.setCursor(20, 156);
  tft.setTextColor(C_YELLOW);
  tft.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20)
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi OK: " + WiFi.localIP().toString());
    // Clear old WiFi message and show success
    tft.fillRect(20, 156, 200, 16, C_BG);
    tft.setTextColor(C_GREEN);
    tft.setCursor(20, 156);
    tft.print("WiFi: ");
    tft.print(WiFi.localIP().toString());

    Serial.println("[INFO] WiFi connected");

    configTime(TIMEZONE_UTC_OFFSET_SECONDS, 0, "pool.ntp.org", "time.google.com");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 2000))
      Serial.println("[NTP] Time set");
    else
      Serial.println("[NTP] Failed to get NTP time");
  }
  else
  {
    Serial.println("\nWiFi offline — running standalone.");
    // Clear old WiFi message and show offline status
    tft.fillRect(20, 156, 200, 16, C_BG);
    tft.setTextColor(C_ORANGE);
    tft.setCursor(20, 156);
    tft.print("WiFi offline — standalone mode.");
  }

  // Web server routes
  server.on("/", handleRoot);
  server.on("/api/log", handleLogJSON);
  server.on("/api/clearlogs", HTTP_POST, handleClearLogs);
  server.on("/api/cards", HTTP_GET, handleCardsJSON);
  server.on("/api/cards", HTTP_POST, handleCardModify);
  server.on("/api/open", HTTP_POST, handleOpenDoor);
  server.on("/update", HTTP_POST, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); }, []()
            {
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if(!upload.filename.endsWith(".bin")){
        Serial.println("Error: Only .bin files are allowed");
        return;
      }
      if(!Update.begin()){//start with max available size
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_WRITE){
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_END){
      if(Update.end(true)){ //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    } });
  server.begin();

  // Update setup
  ArduinoOTA.setHostname("Door");
  ArduinoOTA.setPassword("admin"); // Optional password for security
  ArduinoOTA.onStart([]()
                     {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
    setBacklightForced(true);  // Force backlight on for update progress visibility
    tft.fillScreen(C_BG);
    tft.setTextColor(C_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(20, 100);
    tft.print("Update Starting..."); });
  ArduinoOTA.onEnd([]()
                   {
    Serial.println("\nEnd");
    setBacklightForced(false);  // Release forced backlight
    tft.fillScreen(C_BG);
    tft.setTextColor(C_GREEN);
    tft.setTextSize(2);
    tft.setCursor(20, 100);
    tft.print("Update Complete!");
    delay(2000);
    drawIdle(); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    tft.fillRect(20, 120, 200, 20, C_BG);
    tft.drawRect(20, 120, 200, 20, C_WHITE);
    tft.fillRect(20, 120, (progress * 200) / total, 20, C_GREEN); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    setBacklightForced(false);  // Release forced backlight on error
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED);
    tft.setTextSize(2);
    tft.setCursor(20, 100);
    tft.print("OTA Update Failed!");
    delay(3000);
    drawIdle(); });
  ArduinoOTA.begin();
  Serial.println("OTA ready. Use Arduino IDE to upload over network.");

  delay(1200);
  drawIdle();
  currentScreen = SCR_IDLE;
  lastActivity = millis();
  Serial.println("System ready. Scan card or touch screen.");
}

// =================================================================
//  MAIN LOOP
// =================================================================
void loop()
{
  ArduinoOTA.handle();

  // ── Prioritize physical interactions for fast response ───────

  // ── Exit button unlock ──────────────────────────────────
  int rawExitState = digitalRead(EXIT_BUTTON_PIN);
  if (rawExitState != exitButtonLastRawState)
  {
    exitButtonDebounceTime = millis();
    exitButtonLastRawState = rawExitState;
  }
  if (millis() - exitButtonDebounceTime > EXIT_BUTTON_STABLE_MS)
  {
    if (exitButtonLastStableState == HIGH && rawExitState == LOW)
    {
      // Button pressed event
      exitButtonLastStableState = LOW;
      unsigned long now = millis();
      // ── DEBOUNCE: Only log if enough time has passed since last exit button log
      if ((currentScreen == SCR_IDLE || screenSleeping) && !doorUnlocked && !showingMessage &&
          (now - lastExitButtonLog) >= EXIT_BUTTON_DEBOUNCE_LOG_MS)
      {
        beepSuccess();
        addLog("EXITED", "Exited", "Exit", true);
        lastExitButtonLog = now; // Track when we last logged the exit button
        drawGranted("Exited", "Unlocking");
        unlockDoor("exit button");
        doorUnlocked = true;
        unlockUntil = millis() + UNLOCK_MS;
        showingMessage = true;
        messageText = "Exit unlocked";
        messageTimeout = millis() + MSG_MS;
        backlightOn("exit button pressed"); // Wake screen as side effect
      }
    }
    else if (exitButtonLastStableState == LOW && rawExitState == HIGH)
    {
      exitButtonLastStableState = HIGH;
    }
  }

  // ── Auto relock after RFID unlock period ─────────────────────
  if (doorUnlocked && millis() >= unlockUntil)
  {
    lockDoor("RFID unlock timeout");
    if (digitalRead(RELAY_PIN) != LOCK_CLOSE)
    {
      Serial.println("[AUTOLOCK] Relay state mismatch after first lock attempt, retrying...");
      digitalWrite(RELAY_PIN, LOCK_CLOSE);
      delay(50);
      int retryState = digitalRead(RELAY_PIN);
      Serial.printf("[AUTOLOCK] Retry GPIO%d = %s\n", RELAY_PIN, retryState == LOW ? "LOW" : "HIGH");
    }
    doorUnlocked = false;
    lastScannedUID = "";
    lastCardLeftRange = 0;
  }
  if (showingMessage && millis() >= messageTimeout && currentScreen != SCR_SCAN_NEW)
  {
    showingMessage = false;
    // Special handling for emergency unlock
    if (messageText == "Door unlocking...")
    {
      Serial.println("Door locked - returning to management menu");
      currentScreen = SCR_ADMIN_MENU;
      drawAdminMenu();
    }
    else if (currentScreen == SCR_ADMIN_PIN && messageText.indexOf("Wrong PIN") >= 0)
    {
      drawAdminPin(); // Redraw pin entry
    }
    else if (currentScreen == SCR_ADMIN_MENU)
    {
      drawAdminMenu(); // Redraw menu
    }
    else if (currentScreen == SCR_IDLE || currentScreen == SCR_GRANTED || currentScreen == SCR_DENIED || currentScreen == SCR_BLOCKED)
    {
      currentScreen = SCR_IDLE;
      drawIdle();
    }
  }
  else if (showingMessage && currentScreen == SCR_ADMIN_MENU)
  {
    // Show message bar at bottom
    tft.fillRect(0, 228, 320, 12, C_BG);
    tft.setTextColor(C_CYAN);
    tft.setTextSize(1);
    tft.setCursor(8, 228);
    tft.print(messageText);
  }

  // ── Screen saver / wake logic ───────────────────────────────
  if (!screenSleeping && millis() - lastActivity >= SCREENSAVER_MS)
  {
    drawScreenSaver();
  }

  // ── Touch input ─────────────────────────────────────────────
  int tx, ty;

  if (screenSleeping)
  {
    if (getTouchPoint(tx, ty))
    {
      backlightOn("touch wakes screen");
      drawCurrentScreen();
      firstTouchAfterWake = true;
      touchWasDown = true;
      Serial.println("[WAKE] Screen awakened by touch, release before next tap");
      return;
    }
  }
  else
  {
    if (firstTouchAfterWake)
    {
      if (ts.touched() || touchWasDown)
      {
        if (!ts.touched())
        {
          firstTouchAfterWake = false;
          touchWasDown = false;
        }
        return;
      }
      firstTouchAfterWake = false;
    }

    if (getTouchPoint(tx, ty))
    {
      lastActivity = millis();

      if (showingMessage)
      {
        showingMessage = false;
      }

      beepBuzzer();
      handleTouch(tx, ty);
    }
  }

  // ── Waiting to register or edit a card ─────────────────────
  if ((waitingForNewCard || waitingForEditCard || waitingForBlockCard) &&
      (currentScreen == SCR_SCAN_NEW || currentScreen == SCR_BLOCK_CARD))
  {
    unsigned long now = millis();
    if (now - lastRFIDScan > RFID_SCAN_TIMEOUT && rfid.PICC_IsNewCardPresent())
    {
      lastRFIDScan = now;
      if (rfid.PICC_ReadCardSerial())
      {
        String uid = getUID();
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();

        int idx = findCard(uid);
        if (idx >= 0)
        {
          if (waitingForEditCard)
          {
            waitingForEditCard = false;
            waitingForNewCard = false;
            editExistingMode = true;
            loadEditCard(idx);
            return;
          }
          else if (waitingForBlockCard)
          {
            waitingForBlockCard = false;
            waitingForNewCard = false;
            waitingForEditCard = false;
            loadBlockCard(idx);
            return;
          }

          // Already in database during new registration
          tft.setTextColor(C_YELLOW);
          tft.setTextSize(1);
          tft.setCursor(20, 200);
          tft.print("Card already registered!");
          showingMessage = true;
          messageTimeout = millis() + 1200;
          return;
        }

        if (waitingForEditCard)
        {
          tft.setTextColor(C_RED);
          tft.setTextSize(1);
          tft.setCursor(20, 200);
          tft.print("Card not found for edit.");
          showingMessage = true;
          messageTimeout = millis() + 1200;
          return;
        }
        else if (waitingForBlockCard)
        {
          tft.setTextColor(C_RED);
          tft.setTextSize(1);
          tft.setCursor(20, 200);
          tft.print("Card not found for block/unblock.");
          showingMessage = true;
          messageTimeout = millis() + 1200;
          return;
        }

        if (cardCount < MAX_CARDS)
        {
          cards[cardCount] = {uid, "New User " + String(cardCount + 1), "Staff", false};
          editCardIndex = cardCount;
          cardCount++;
          saveCardsToSPIFFS(); // ← Save to SPIFFS immediately
          waitingForNewCard = false;
          waitingForEditCard = false;
          editExistingMode = false;
          loadEditCard(editCardIndex);
          return;
        }

        tft.setTextColor(C_RED);
        tft.setTextSize(1);
        tft.setCursor(20, 200);
        tft.print("Card database full! Max 20.");
        showingMessage = true;
        messageTimeout = millis() + 1200;
        return;
      }
    }

    // ── Handle card added screen timeout ────────────────────────
    if (showingMessage && (currentScreen == SCR_SCAN_NEW || currentScreen == SCR_BLOCK_CARD) && messageTimeout <= millis())
    {
      currentScreen = SCR_ADMIN_MENU;
      drawAdminMenu();
      showingMessage = false;
    }
  }

  // ── Normal RFID scan (only from idle screen) ─────────────────
  static unsigned long rfidFailureCount = 0;
  if ((currentScreen == SCR_IDLE || screenSleeping) && !doorUnlocked)
  {
    unsigned long now2 = millis();
    if (now2 - lastRFIDScan >= RFID_SCAN_TIMEOUT)
    {
      lastRFIDScan = now2;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
      {
        rfidFailureCount = 0; // Reset on success
        String uid = getUID();
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();

        // ── Wake screen if sleeping before showing result ─────────────────
        if (screenSleeping)
        {
          backlightOn("RFID scanned");
          Serial.println("[WAKE] Screen awakened by RFID scan");
        }

        // ── DEBOUNCE: Only log if this is a NEW card (different from last scanned)
        bool isNewCard = (uid != lastScannedUID);
        if (!isNewCard && (now2 - lastCardLeftRange) < CARD_DEBOUNCE_MS)
        {
          // Same card still in range, don't log again
          Serial.printf("[RFID] Same card %s still in range (debounced)\n", uid.c_str());
          return;
        }

        lastScannedUID = uid; // Track current card
        int idx = findCard(uid);

        if (idx >= 0 && cards[idx].blocked)
        {
          // BLOCKED
          currentScreen = SCR_BLOCKED;
          lastBlockedName = cards[idx].name;
          beepFailure();
          addLog(uid, cards[idx].name, cards[idx].role, false);
          drawBlocked(cards[idx].name);
          showingMessage = true;
          messageTimeout = millis() + MSG_MS;
          messageText = "Blocked";
        }
        else if (idx >= 0)
        {
          // GRANTED
          currentScreen = SCR_GRANTED;
          lastGrantedName = cards[idx].name;
          lastGrantedRole = cards[idx].role;
          beepSuccess();
          addLog(uid, cards[idx].name, cards[idx].role, true);
          drawGranted(cards[idx].name, cards[idx].role);
          unlockDoor("RFID granted"); // ← centralised call
          doorUnlocked = true;
          unlockUntil = millis() + UNLOCK_MS;
          showingMessage = true;
          messageTimeout = millis() + MSG_MS;
          messageText = "Granted";
        }
        else
        {
          // DENIED — unknown card
          currentScreen = SCR_DENIED;
          lastDeniedUID = uid;
          beepFailure();
          addLog(uid, "Unknown", "N/A", false);
          drawDenied(uid);
          showingMessage = true;
          messageTimeout = millis() + MSG_MS;
          messageText = "Denied";
        }
      }
      else
      {
        // ── NO CARD DETECTED: Check if one just left the range
        if (lastScannedUID != "")
        {
          lastCardLeftRange = now2;
          lastScannedUID = ""; // Clear last scanned UID
          Serial.println("[RFID] Card left reader range");
        }
        else
        {
          // Increment failure count if no card and no previous card
          rfidFailureCount++;
          if (rfidFailureCount > 10) // After 10 consecutive failures, reset
          {
            Serial.println("[RFID] Reader unresponsive, resetting PCD...");
            rfid.PCD_Init();
            rfidFailureCount = 0;
          }
        }
      }
    }
  }

  // ── Background tasks (can take time, low priority) ───────────

  server.handleClient();

  // ── Periodic SPIFFS backup (every 60 seconds) ────────────────
  static unsigned long lastSPIFFSSave = 0;
  unsigned long now = millis();
  if (now - lastSPIFFSSave > 60000) // 60 second interval
  {
    lastSPIFFSSave = now;
    saveLogToSPIFFS();
  }
}
