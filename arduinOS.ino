#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>  // For hardware timer-based sound
#include <esp_heap_caps.h>  // For PSRAM allocation

// Custom GFXcanvas16 that tries PSRAM first, falls back to regular SRAM
// ESP32-S3 has 8MB PSRAM, ESP32-WROOM-32 has only ~320KB internal SRAM
class PSRAMCanvas16 : public Adafruit_GFX {
public:
  PSRAMCanvas16(uint16_t w, uint16_t h) : Adafruit_GFX(w, h) {
    // Try PSRAM first (if available), fall back to regular malloc
    #ifdef BOARD_HAS_PSRAM
    buffer = (uint16_t*)heap_caps_malloc(w * h * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    #endif
    if (!buffer) {
      buffer = (uint16_t*)malloc(w * h * sizeof(uint16_t));
    }
    if (buffer) memset(buffer, 0, w * h * sizeof(uint16_t));
  }
  ~PSRAMCanvas16() {
    if (buffer) free(buffer);
  }
  uint16_t* getBuffer() { return buffer; }
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (buffer && x >= 0 && x < _width && y >= 0 && y < _height) {
      buffer[y * _width + x] = color;
    }
  }
  void fillScreen(uint16_t color) override {
    if (buffer) {
      uint32_t n = _width * _height;
      // Optimize for common colors (black = 0x0000)
      if (color == 0x0000) {
        memset(buffer, 0, n * sizeof(uint16_t));
      } else {
        // Use 32-bit writes for speed (2 pixels at a time)
        uint32_t color32 = ((uint32_t)color << 16) | color;
        uint32_t* buf32 = (uint32_t*)buffer;
        uint32_t n32 = n / 2;
        for (uint32_t i = 0; i < n32; i++) buf32[i] = color32;
        if (n & 1) buffer[n - 1] = color;  // Handle odd pixel
      }
    }
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    // Clip to screen bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _width) w = _width - x;
    if (y + h > _height) h = _height - y;
    if (w <= 0 || h <= 0 || !buffer) return;
    
    // Optimized row-by-row fill
    for (int16_t row = y; row < y + h; row++) {
      uint16_t* rowPtr = buffer + row * _width + x;
      for (int16_t col = 0; col < w; col++) {
        rowPtr[col] = color;
      }
    }
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    if (y < 0 || y >= _height || !buffer) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > _width) w = _width - x;
    if (w <= 0) return;
    uint16_t* ptr = buffer + y * _width + x;
    for (int16_t i = 0; i < w; i++) ptr[i] = color;
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    if (x < 0 || x >= _width || !buffer) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > _height) h = _height - y;
    if (h <= 0) return;
    for (int16_t i = 0; i < h; i++) buffer[(y + i) * _width + x] = color;
  }
private:
  uint16_t* buffer = nullptr;
};

// Helper to allocate from PSRAM if available, else regular malloc
inline void* psram_malloc(size_t size) {
  void* ptr = nullptr;
  #ifdef BOARD_HAS_PSRAM
  ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  #endif
  if (!ptr) ptr = malloc(size);
  return ptr;
}

// WiFi and NTP
Preferences prefs;
char wifiSSID[33] = "";
char wifiPass[65] = "";

// BIOS settings (stored in ESP32 NVS, not SD card)
bool biosRequireSD = true;      // Require SD card at boot
bool biosEnableWiFi = true;     // Enable WiFi at boot
bool wifiConnected = false;
bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;
bool wifiScanning = false;
bool ntpSyncing = false;
unsigned long ntpSyncStart = 0;
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -7 * 3600;  // UTC-7 (adjust for your timezone)
const int daylightOffset_sec = 3600;   // DST offset

// Uncomment these for smoother fonts (slower rendering)
// #include <Fonts/FreeSans9pt7b.h>
// #include <Fonts/FreeSansBold12pt7b.h>
// #include <Fonts/FreeSansBold24pt7b.h>
#define USE_BUILTIN_FONTS 1

// Pin definitions for ST7789 (ESP32 VSPI)
#define TFT_CS    5
#define TFT_RST   17
#define TFT_DC    16
#define TFT_BLK   22  // Backlight control pin (adjust if different on your board)
// SCK = GPIO 18 (hardware SPI)
// MOSI = GPIO 23 (hardware SPI)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// SD Card pins (ESP32 HSPI - separate from display)
#define SD_CS     21
#define SD_SCK    13
#define SD_MOSI   15
#define SD_MISO   35
SPIClass hspi(HSPI);
bool sdCardMounted = false;
bool sdCardRemoved = false;  // Hot-swap detection state

// Joystick pins (active LOW, connect COM to GND)
#define JOY_UP    32
#define JOY_DOWN  33
#define JOY_LEFT  25
#define JOY_RIGHT 26
#define JOY_MID   27
#define JOY_SET   14
#define JOY_RST   12
#define BUZZER_PIN 4

// Screen dimensions (landscape)
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 135

// Double buffer for all modes (64KB) - allocated once at startup in PSRAM, never freed
PSRAMCanvas16* deskBuffer = nullptr;
Adafruit_GFX* gfx = nullptr;  // Points to either deskBuffer or tft
bool bufferReady = false;  // True when buffer is allocated and valid

// Safe buffer initialization - called lazily when entering desktop/game mode
// Returns true if buffer is ready, false if allocation failed
bool initDoubleBuffer() {
  if (bufferReady && deskBuffer && deskBuffer->getBuffer()) {
    return true;  // Already allocated
  }
  
  if (deskBuffer == nullptr) {
    yield();  // Let system settle before large allocation
    deskBuffer = new PSRAMCanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);
    yield();
  }
  
  if (deskBuffer && deskBuffer->getBuffer()) {
    bufferReady = true;
    return true;
  }
  
  // Allocation failed - clean up
  bufferReady = false;
  if (deskBuffer) {
    delete deskBuffer;
    deskBuffer = nullptr;
  }
  return false;
}

// Track when we last did a full TFT reinit
unsigned long lastTFTReinit = 0;
#define TFT_REINIT_INTERVAL 200  // Full reinit every 200ms (aggressive test)
bool tftNeedsReinit = false;     // Flag to trigger reinit in main loop

// Ensure TFT controller state is correct before any TFT operation
// This is now lightweight - full reinit happens in main loop
void ensureTFTState() {
  digitalWrite(SD_CS, HIGH);    // Ensure SD card is deselected
  delayMicroseconds(50);        // Let SPI bus settle
  tft.setRotation(3);           // Quick state restore
  tft.setSPISpeed(20000000);
}

// Periodic TFT state refresh - restores controller config without full init
// This avoids the black flash that tft.init() causes
void periodicTFTReinit() {
  unsigned long now = millis();
  if (now - lastTFTReinit >= TFT_REINIT_INTERVAL) {
    lastTFTReinit = now;
    digitalWrite(SD_CS, HIGH);  // Ensure SD card deselected
    delayMicroseconds(50);
    // Just restore state without full init (no black flash)
    tft.setRotation(3);
    tft.setSPISpeed(20000000);
    // Send ST7789 commands to ensure display is on and configured correctly
    tft.enableDisplay(true);
    tft.invertDisplay(true);  // ST7789 requires inversion for correct colors
  }
}

// Safe buffer flush to screen - handles all error cases
void flushBuffer() {
  if (!bufferReady || !deskBuffer || !deskBuffer->getBuffer()) {
    // If drawing directly to tft (no buffer), nothing to flush
    return;
  }
  
  // CRITICAL: Restore TFT state before transfer
  ensureTFTState();
  
  // Single atomic transfer - faster and more reliable than chunked
  uint16_t* buf = deskBuffer->getBuffer();
  tft.drawRGBBitmap(0, 0, buf, SCREEN_WIDTH, SCREEN_HEIGHT);
}

// Safe partial buffer flush (for game apps that only update part of screen)
void flushBufferRegion(int16_t startY, int16_t h) {
  if (!bufferReady || !deskBuffer || !deskBuffer->getBuffer()) {
    return;
  }
  if (startY < 0) startY = 0;
  if (startY + h > SCREEN_HEIGHT) h = SCREEN_HEIGHT - startY;
  if (h <= 0) return;
  
  // CRITICAL: Restore TFT state before transfer
  ensureTFTState();
  
  // Single atomic transfer for the region
  uint16_t* buf = deskBuffer->getBuffer();
  tft.drawRGBBitmap(0, startY, buf + (startY * SCREEN_WIDTH), SCREEN_WIDTH, h);
}

// ArduinOS logo bitmap (75x30px, 1-bit monochrome)
const unsigned char epd_bitmap_logoarduin [] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xff, 0x3f, 0xe0, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0x9c, 0x78, 0xef, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3e, 0x79, 0xf7, 0xe0, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3e, 0x31, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0x7e, 0x11, 0xef, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7e, 0x18, 0xdf, 0xe0, 0xff, 0xff, 
  0xfe, 0xff, 0xff, 0xfc, 0x7e, 0x18, 0x3f, 0xe0, 0xfe, 0xff, 0xfe, 0xff, 0xdf, 0xf8, 0xfe, 0x1c, 
  0x3f, 0xe0, 0xfe, 0x7f, 0xfe, 0xff, 0xdf, 0xf8, 0xfe, 0x1c, 0x3f, 0xe0, 0xff, 0x7f, 0xfe, 0xff, 
  0xff, 0xf8, 0xfe, 0x1e, 0x1f, 0xe0, 0xfd, 0x3a, 0x76, 0xcd, 0xdd, 0x18, 0xfe, 0x3f, 0x1f, 0xe0, 
  0xfd, 0x39, 0x6e, 0xcd, 0xdc, 0x98, 0xfe, 0x3f, 0x0f, 0xe0, 0xfb, 0xbb, 0x4e, 0xdd, 0xdd, 0x98, 
  0xfe, 0x7f, 0x8f, 0xe0, 0xf8, 0x3b, 0xdd, 0xd9, 0xd9, 0x98, 0xfc, 0x77, 0x8f, 0xe0, 0xf7, 0xbb, 
  0xdd, 0xd9, 0xd9, 0xbc, 0xfc, 0xe3, 0x8f, 0xe0, 0xf7, 0x9b, 0xc9, 0xc5, 0xd9, 0xbc, 0x79, 0xc3, 
  0x9f, 0xe0, 0xf7, 0xdb, 0xe7, 0xcd, 0xdb, 0xbe, 0x33, 0xe0, 0x3f, 0xe0, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0x8f, 0xf8, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0
};
#define LOGO_WIDTH 75
#define LOGO_HEIGHT 30

// UI Layout
#define MENUBAR_HEIGHT 18
#define APP_ROW_HEIGHT 32
#define APP_ICON_SIZE  24
#define APP_PADDING    6

// Color scheme (RGB565) - Non-desktop mode (original dark theme)
#define COLOR_BG        0x0000  // Black background
#define COLOR_MENUBAR   0x18E3  // Dark gray menubar
#define COLOR_TEXT      0xFFFF  // White text
#define COLOR_ACCENT    0x07FF  // Cyan accent
#define COLOR_HIGHLIGHT 0x2124  // Selection highlight (darker)
#define COLOR_ICON_BG   0x3186  // Icon background

// 3D effect colors for beveled edges (desktop mode)
#define COLOR_3D_LIGHT  0xFFFF  // White highlight (top/left edges)
#define COLOR_3D_DARK   0x8410  // Dark gray shadow (bottom/right edges)
#define COLOR_3D_FACE   0xC618  // Button/window face color

// App icon colors
#define COLOR_SETTINGS  0x7BEF  // Gray
#define COLOR_CALC      0xFD20  // Orange
#define COLOR_TIMER     0xF800  // Red
#define COLOR_MESSAGES  0x07E0  // Green
#define COLOR_MUSIC     0xF81F  // Magenta

// App IDs
#define APP_DESKTOP   0
#define APP_FILES     1
#define APP_SETTINGS  2
#define APP_CALC      3
#define APP_TIMER     4
#define APP_TODO      5
#define APP_NOTES     6
#define APP_MUSIC     7
#define APP_PAINT     8
#define APP_SNAKE     9
#define APP_TETRIS    10
#define APP_RESTART   11

// App definitions
#define NUM_APPS 12
const char* appNames[NUM_APPS] = {
  "Desktop",
  "Files",
  "Settings",
  "Calc",
  "Timer",
  "TODO",
  "Notes",
  "Music",
  "Paint",
  "Snake",
  "Tetris",
  "Restart"
};
#define COLOR_DESKTOP 0x001F  // Blue (Win98 style)
#define COLOR_FILES   0xFFE0  // Yellow (folder color)
#define COLOR_PAINT   0xF81F  // Magenta
#define COLOR_SNAKE   0x07E0  // Green
#define COLOR_TETRIS  0x07FF  // Cyan
#define COLOR_RESTART 0xF800  // Red
const uint16_t appColors[NUM_APPS] = {
  COLOR_DESKTOP, COLOR_FILES, COLOR_SETTINGS, COLOR_CALC, COLOR_TIMER,
  COLOR_MESSAGES, COLOR_MESSAGES, COLOR_MUSIC, COLOR_PAINT, COLOR_SNAKE, COLOR_TETRIS, COLOR_RESTART
};

// UI State
enum Screen { SCREEN_HOME, SCREEN_DRAWER, SCREEN_APP, SCREEN_SWITCHER };
Screen currentScreen = SCREEN_HOME;
int selectedApp = 0;
int scrollOffset = 0;
int maxVisibleApps;

// App switcher - tracks which apps are "open"
bool appOpen[NUM_APPS] = {false, false, false, false, false, false, false, false, false, false, false, false};
int switcherSel = 0;

// Notification system
bool notifActive = false;
char notifText[20] = "";
unsigned long notifTime = 0;

// MID button timing for hold detection
unsigned long midPressStart = 0;
bool midWasHeld = false;

// Time (placeholder - would use RTC in real implementation)
int hours = 12;
int minutes = 30;
int seconds = 0;
int day = 15;
int month = 12;
int year = 2025;
int lastDisplayedMinute = 30;  // Start matching initial minutes

// Input state
bool lastMidState = HIGH;
bool lastUpState = HIGH;
bool lastDownState = HIGH;
bool lastLeftState = HIGH;
bool lastRightState = HIGH;
bool lastSetState = HIGH;

// Timing
unsigned long lastTime;
unsigned long lastSecond;

// Previous selection for partial redraw
int prevSelectedApp = -1;

// Track which button was just pressed for apps
int lastButton = 0;  // 0=none, 1=up, 2=down, 3=left, 4=right, 5=mid

// ===== APP STATE VARIABLES =====

// Sound functions for ESP32 (uses LEDC PWM with hardware timer for non-blocking)
bool soundEnabled = true;

// Hardware timer-based sound system - runs independently of main loop
esp_timer_handle_t soundTimer = NULL;
volatile bool soundPlaying = false;
volatile unsigned long lastSoundEnd = 0;
#define SOUND_GAP_MS 15  // Minimum gap between sounds to make them distinct

// Timer callback - runs in interrupt context to stop sound
void IRAM_ATTR soundTimerCallback(void* arg) {
  ledcWriteTone(BUZZER_PIN, 0);
  ledcWrite(BUZZER_PIN, 0);
  soundPlaying = false;
  lastSoundEnd = millis();
}

void stopSound() {
  if (soundTimer) {
    esp_timer_stop(soundTimer);
  }
  ledcWriteTone(BUZZER_PIN, 0);
  ledcWrite(BUZZER_PIN, 0);
  soundPlaying = false;
  lastSoundEnd = millis();
}

// updateSound() is now a no-op since timer handles everything
void updateSound() {
  // Timer-based system handles sound automatically
  // This function kept for compatibility but does nothing
}

void initSoundTimer() {
  esp_timer_create_args_t timerArgs = {
    .callback = soundTimerCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "sound_timer"
  };
  esp_timer_create(&timerArgs, &soundTimer);
}

void startTone(int freq, int duration) {
  if (soundEnabled && freq > 0) {
    unsigned long now = millis();
    
    // If a sound is currently playing, stop it first
    if (soundPlaying) {
      stopSound();
    }
    
    // Enforce minimum gap between sounds to make them distinct
    if (now - lastSoundEnd < SOUND_GAP_MS) {
      return;  // Skip this sound - too close to previous one
    }
    
    // Cap duration at 50ms for click sounds to keep them short and distinct
    if (duration > 50) duration = 50;
    
    // Start the tone
    ledcWriteTone(BUZZER_PIN, freq);
    soundPlaying = true;
    
    // Schedule timer to stop the sound (duration in microseconds)
    if (soundTimer) {
      esp_timer_stop(soundTimer);  // Stop any pending timer
      esp_timer_start_once(soundTimer, duration * 1000);  // Convert ms to us
    }
  } else {
    stopSound();  // Ensure buzzer is off if sound disabled
  }
}

void espTone(int freq, int duration) {
  // Now uses timer-based system, no blocking needed
  startTone(freq, duration);
}

void playTick() { startTone(150, 10); }
void playSelect() { startTone(200, 20); }
void playBack() { startTone(100, 20); }
void playAlert() { startTone(800, 400); }
void playPopup() { startTone(300, 30); }
void playError() { startTone(200, 160); }  // Simplified - single tone
void playClick() { startTone(180, 8); }

// TFT recovery - reinitialize display if it goes black
void recoverTFT() {
  digitalWrite(SD_CS, HIGH);  // Ensure SD card deselected first
  delay(10);
  tft.init(135, 240, SPI_MODE3);
  tft.setRotation(3);
  tft.setSPISpeed(20000000);
  digitalWrite(TFT_BLK, HIGH);
}

// SD card hot-swap check - returns true if card is present
bool checkSDCard() {
  // Try to open root directory - more reliable than cardType()
  File root = SD.open("/");
  
  // CRITICAL: Full TFT SPI restore after SD card access
  digitalWrite(SD_CS, HIGH);  // Ensure SD card is deselected
  delayMicroseconds(10);      // Brief delay for SPI bus to settle
  tft.setSPISpeed(20000000);  // Restore TFT SPI speed
  
  if (root) {
    root.close();
    return true;  // Card is accessible
  }
  return false;  // Card is not accessible
}

// Verify SD card is accessible before operations - attempts remount if needed
bool ensureSDCard() {
  if (!sdCardMounted) return false;
  
  // Try to open root directory - more reliable check
  File root = SD.open("/");
  
  // CRITICAL: Always restore TFT after SD card access
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(10);
  tft.setSPISpeed(20000000);
  
  if (root) {
    root.close();
    return true;  // Card is fine
  }
  
  // Card seems disconnected - try immediate remount
  Serial.println("SD card check failed, attempting remount...");
  SD.end();
  delay(50);
  if (SD.begin(SD_CS, hspi)) {
    sdCardMounted = true;
    sdCardRemoved = false;
    Serial.println("SD card remounted successfully");
    // CRITICAL: Full TFT reinit after SD remount
    digitalWrite(SD_CS, HIGH);
    delay(20);
    tft.init(135, 240, SPI_MODE3);
    tft.setRotation(3);
    tft.setSPISpeed(20000000);
    return true;
  }
  
  // Remount failed - still need to restore TFT
  sdCardMounted = false;
  digitalWrite(SD_CS, HIGH);
  delay(10);
  tft.setSPISpeed(20000000);
  
  // Mark SD card as removed - the main loop will handle the popup
  if (biosRequireSD) {
    sdCardRemoved = true;
  }
  
  return false;
}

// Attempt to remount SD card
bool remountSDCard() {
  SD.end();
  delay(20);  // Reduced from 100ms
  if (SD.begin(SD_CS, hspi)) {
    sdCardMounted = true;
    sdCardRemoved = false;
    // CRITICAL: Full TFT reinit after SD card SPI operations
    digitalWrite(SD_CS, HIGH);  // Ensure SD card is deselected
    delay(20);  // Reduced from 50ms
    tft.init(135, 240, SPI_MODE3);
    tft.setRotation(3);
    tft.setSPISpeed(20000000);
    return true;
  }
  // CRITICAL: Restore TFT SPI even on failure
  digitalWrite(SD_CS, HIGH);  // Ensure SD card is deselected
  delayMicroseconds(10);
  tft.setSPISpeed(20000000);
  return false;
}

// Load BIOS settings from ESP32 NVS
void loadBiosSettings() {
  prefs.begin("bios", true);  // Read-only
  biosRequireSD = prefs.getBool("requireSD", true);
  biosEnableWiFi = prefs.getBool("enableWiFi", true);
  prefs.end();
}

// Save BIOS settings to ESP32 NVS
void saveBiosSettings() {
  prefs.begin("bios", false);  // Read-write
  prefs.putBool("requireSD", biosRequireSD);
  prefs.putBool("enableWiFi", biosEnableWiFi);
  prefs.end();
}

// BIOS setup screen - called when SET is held during boot (with double buffering)
void showBiosScreen() {
  int biosSel = 0;
  const int BIOS_ITEMS = 2;
  bool lastUp = HIGH, lastDown = HIGH, lastMid = HIGH, lastSet = HIGH;
  unsigned long biosEntryTime = millis();  // Track when we entered BIOS
  
  // Allocate buffer for BIOS screen
  GFXcanvas16* biosBuffer = new GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);
  if (!biosBuffer) {
    // Fallback to direct drawing if allocation fails
    tft.fillScreen(0x0000);
    tft.setTextColor(0xF800);
    tft.setCursor(50, 60);
    tft.print("BIOS Memory Error");
    delay(2000);
    return;
  }
  
  while (true) {
    // Draw BIOS screen to buffer
    biosBuffer->fillScreen(0x0000);  // Black background
    
    // Title bar
    biosBuffer->fillRect(0, 0, SCREEN_WIDTH, 16, 0x07FF);  // Cyan
    biosBuffer->setTextSize(1);
    biosBuffer->setTextColor(0x0000);
    biosBuffer->setCursor(70, 4);
    biosBuffer->print("ArduinOS BIOS Setup");
    
    // Settings list
    int itemH = 20;
    int startY = 25;
    
    for (int i = 0; i < BIOS_ITEMS; i++) {
      int y = startY + i * itemH;
      bool sel = (i == biosSel);
      
      if (sel) {
        biosBuffer->fillRect(0, y, SCREEN_WIDTH, itemH, 0x4208);  // Dark gray highlight
      }
      
      biosBuffer->setTextColor(0x07FF);  // Cyan text
      biosBuffer->setCursor(8, y + 6);
      
      if (i == 0) {
        biosBuffer->print("Require SD card");
        // Value
        biosBuffer->setCursor(180, y + 6);
        biosBuffer->setTextColor(biosRequireSD ? 0x07E0 : 0xF800);  // Green/Red
        biosBuffer->print(biosRequireSD ? "YES" : "NO");
      } else if (i == 1) {
        biosBuffer->print("Enable WiFi at Boot");
        // Value
        biosBuffer->setCursor(180, y + 6);
        biosBuffer->setTextColor(biosEnableWiFi ? 0x07E0 : 0xF800);
        biosBuffer->print(biosEnableWiFi ? "YES" : "NO");
      }
    }
    
    // Instructions
    biosBuffer->setTextColor(0x7BEF);  // Gray
    biosBuffer->setCursor(10, SCREEN_HEIGHT - 30);
    biosBuffer->print("UP/DOWN: Navigate   MID: Toggle");
    biosBuffer->setCursor(10, SCREEN_HEIGHT - 18);
    biosBuffer->print("SET: Save & Exit");
    
    // Version info
    biosBuffer->setTextColor(0x4208);
    biosBuffer->setCursor(10, SCREEN_HEIGHT - 6);
    biosBuffer->print("BIOS v1.0 - ESP32 NVS Storage");
    
    // Push buffer to display
    yield();
    ensureTFTState();  // Restore TFT state before transfer
    noInterrupts();
    tft.drawRGBBitmap(0, 0, biosBuffer->getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
    interrupts();
    yield();
    
    delay(50);  // Debounce
    updateSound();  // Update non-blocking sound
    
    // Read inputs
    bool up = digitalRead(JOY_UP);
    bool down = digitalRead(JOY_DOWN);
    bool mid = digitalRead(JOY_MID);
    bool set = digitalRead(JOY_SET);
    
    // Navigate up
    if (up == LOW && lastUp == HIGH) {
      if (biosSel > 0) biosSel--;
      startTone(150, 10);
    }
    // Navigate down
    if (down == LOW && lastDown == HIGH) {
      if (biosSel < BIOS_ITEMS - 1) biosSel++;
      startTone(150, 10);
    }
    // Toggle value
    if (mid == LOW && lastMid == HIGH) {
      if (biosSel == 0) biosRequireSD = !biosRequireSD;
      else if (biosSel == 1) biosEnableWiFi = !biosEnableWiFi;
      startTone(200, 20);
    }
    // Save and exit (with 1s cooldown to prevent accidental exit)
    if (set == LOW && lastSet == HIGH && millis() - biosEntryTime >= 1000) {
      saveBiosSettings();
      startTone(300, 30);
      delay(200);
      break;
    }
    
    lastUp = up;
    lastDown = down;
    lastMid = mid;
    lastSet = set;
  }
  
  // Free buffer
  delete biosBuffer;
}

// Settings app
int settingsSel = 0;
int settingsScrollOffset = 0;
#define SETTINGS_ITEMS 7  // Sound, WiFi, Hour, Min, BootDesk, Wipe, Version
// WiFi editor state
bool wifiEditMode = false;
bool wifiScanMode = false;  // true = selecting SSID from list, false = entering password
int wifiScanSel = 0;        // Selected network in scan list
int wifiScanCount = 0;      // Number of networks found
#define MAX_WIFI_NETWORKS 10
char wifiNetworks[MAX_WIFI_NETWORKS][33];  // Store SSIDs
int wifiCursor = 0;
int wifiCharIdx = 0;
const char wifiChars[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 !@#$%^&*()-_=+[]{}|;:',.<>?/`~";
#define WIFI_NUM_CHARS 94

// Calculator app
long calcValue = 0;
long calcStored = 0;
int calcOp = 0;
bool calcNew = true;
int calcRow = 0, calcCol = 0;
// Desktop calc variables
int calcNum1 = 0, calcNum2 = 0;
long calcResult = 0;
bool calcHasNum2 = false;
const char calcOps[4] = {'+', '-', '*', '/'};

// Timer app
unsigned long timerEndTime = 0;
unsigned long timerRemaining = 0;  // For pause
unsigned long timerStartTime = 0;  // When timer started (renamed from timerStart to avoid ESP32 conflict)
int timerSetMin = 1, timerSetSec = 0;
int timerSel = 0;
bool timerOn = false;
bool timerPaused = false;

// TODO app
#define MAX_TODO 10  // Maximum number of TODO items
#define TODO_ITEM_LEN 16
char todoItems[MAX_TODO][TODO_ITEM_LEN + 1] = {"Shopping", "Call mom", "Meeting", "Workout", "", "", "", "", "", ""};
bool todoDone[MAX_TODO] = {false, false, false, false, false, false, false, false, false, false};
int todoCount = 4;  // Current number of TODO items
int todoSel = 0;
int todoScrollOffset = 0;
bool todoEditMode = false;  // true = editing text of selected item (non-desktop)
int todoCursor = 0;         // cursor position in text
int todoCharIdx = 0;        // index in character set
// Desktop TODO editing
bool deskTodoEditMode = false;  // true = editing in desktop mode
int deskTodoEditIdx = -1;       // which item is being edited
int deskTodoCursor = 0;         // cursor position in desktop edit

// Notes app - separate from file editor, has its own persistent text
#define NOTE_LEN 60
char notesText[NOTE_LEN + 1] = "";  // Notes app text (saved to .osdata)
int notesCursor = 0;

// Shared keyboard state
int noteCharIdx = 0;
const char noteChars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,!?-";
#define NUM_CHARS 42

// Music app
int musicTrack = 0;
int musicScrollOffset = 0;
#define NUM_TRACKS 5
const char* trackNames[NUM_TRACKS] = {"Twinkle", "Happy BD", "Fur Elise", "Scale", "Alert"};

// File Explorer state
#define MAX_FILES 100
#define MAX_PATH_LEN 128
#define MAX_FILENAME_LEN 64

// Editor app - for editing .txt files (must be after MAX_PATH_LEN)
char editorText[NOTE_LEN + 1] = "";  // Editor text buffer
int editorCursor = 0;
char editorFilePath[MAX_PATH_LEN] = "";  // File being edited
char currentPath[MAX_PATH_LEN] = "/";
char fileNames[MAX_FILES][MAX_FILENAME_LEN];
bool fileIsDir[MAX_FILES];
int fileCount = 0;
int fileScrollOffset = 0;
int fileSelected = 0;
unsigned long lastFilesRefresh = 0;  // For auto-refresh every 1s

// Scrollbar dragging state
bool scrollDragging = false;
int scrollDragApp = -1;  // Which app is being scrolled
bool scrollDragVertical = true;  // true=vertical, false=horizontal

// Rename mode state - uses Editor keyboard for input
bool renameMode = false;
char renameExtension[16] = "";  // Store file extension during rename

// Desktop WiFi connection screen state (wifiEditMode already defined above)
char wifiEditSSID[33] = "";  // SSID being connected to
char wifiEditPass[65] = "";  // Password being entered
int wifiEditCursor = 0;
bool wifiDropdownOpen = false;  // true when dropdown is showing network list
int wifiDropdownScroll = 0;  // Scroll offset for dropdown list

// Virtual keyboard mode: 0=uppercase, 1=lowercase, 2=symbols
int kbMode = 0;

// Global flag to trigger desktop redraw from outside drawDesktopApp
bool deskNeedRedraw = false;

// Cursor hide state (for games)
bool cursorHidden = false;

// Tetris game state
#define TETRIS_COLS 10
#define TETRIS_ROWS 20
uint8_t tetrisBoard[TETRIS_ROWS][TETRIS_COLS];  // 0=empty, 1-7=piece colors
int tetrisPieceX, tetrisPieceY;  // Current piece position
int tetrisPieceType;  // 0-6 for I,O,T,S,Z,J,L
int tetrisPieceRot;   // 0-3 rotation
int tetrisScore;
int tetrisLevel;
int tetrisLines;
bool tetrisGameOver;
bool tetrisPlaying;
bool tetrisShowMenu;  // Show main menu
bool tetrisPaused;    // Game is paused
int tetrisMenuSel;    // Menu selection (0=resume/retry, 1=menu)
unsigned long tetrisLastDrop;
int tetrisDropInterval;  // ms between drops
int tetrisSoftDropInterval;  // Faster drop when holding down
bool tetrisSoftDropping;  // Currently soft dropping
// Tetris piece definitions (4x4 grid, 4 rotations each)
// I, O, T, S, Z, J, L pieces
const uint16_t tetrisPieces[7][4] = {
  {0x0F00, 0x2222, 0x00F0, 0x4444},  // I
  {0xCC00, 0xCC00, 0xCC00, 0xCC00},  // O
  {0x0E40, 0x4C40, 0x4E00, 0x4640},  // T
  {0x06C0, 0x8C40, 0x6C00, 0x4620},  // S
  {0x0C60, 0x4C80, 0xC600, 0x2640},  // Z
  {0x44C0, 0x8E00, 0x6440, 0x0E20},  // J
  {0x4460, 0x0E80, 0xC440, 0x2E00}   // L
};
const uint16_t tetrisColors[7] = {0x07FF, 0xFFE0, 0xF81F, 0x07E0, 0xF800, 0x001F, 0xFD20};  // Cyan, Yellow, Magenta, Green, Red, Blue, Orange

// Tetris theme music (Korobeiniki) - note frequencies and durations
// Format: {frequency, duration_ms} - 0 frequency = rest
#define TETRIS_THEME_LEN 40
const uint16_t tetrisTheme[TETRIS_THEME_LEN][2] = {
  {659, 400}, {494, 200}, {523, 200}, {587, 400}, {523, 200}, {494, 200},  // E5 B4 C5 D5 C5 B4
  {440, 400}, {440, 200}, {523, 200}, {659, 400}, {587, 200}, {523, 200},  // A4 A4 C5 E5 D5 C5
  {494, 400}, {494, 200}, {523, 200}, {587, 400}, {659, 400},              // B4 B4 C5 D5 E5
  {523, 400}, {440, 400}, {440, 400}, {0, 200},                            // C5 A4 A4 rest
  {587, 400}, {698, 200}, {880, 400}, {784, 200}, {698, 200},              // D5 F5 A5 G5 F5
  {659, 600}, {523, 200}, {659, 400}, {587, 200}, {523, 200},              // E5 C5 E5 D5 C5
  {494, 400}, {494, 200}, {523, 200}, {587, 400}, {659, 400},              // B4 B4 C5 D5 E5
  {523, 400}, {440, 400}, {440, 400}, {0, 200}                             // C5 A4 A4 rest
};
int tetrisMusicNote = 0;           // Current note index
unsigned long tetrisMusicLastNote = 0;  // When last note started
bool tetrisMusicPlaying = false;   // Is music currently playing
int tetrisHighScore = 0;           // High score stored in .osdata

// Snake game state
#define SNAKE_COLS 24
#define SNAKE_ROWS 13
#define SNAKE_MAX_LEN 100
int snakeX[SNAKE_MAX_LEN], snakeY[SNAKE_MAX_LEN];  // Snake body positions
int snakeLen;
int snakeDir;  // 0=up, 1=right, 2=down, 3=left
int snakeFoodX, snakeFoodY;
int snakeScore;
bool snakeGameOver;
bool snakePlaying;
bool snakeShowMenu;
bool snakePaused;
int snakeMenuSel;
unsigned long snakeLastMove;
int snakeMoveInterval;
int snakeHighScore = 0;  // High score stored in .osdata

// Snake music - simple catchy loop
#define SNAKE_THEME_LEN 16
const uint16_t snakeTheme[SNAKE_THEME_LEN][2] = {
  {330, 150}, {392, 150}, {440, 150}, {392, 150},  // E4 G4 A4 G4
  {330, 150}, {294, 150}, {330, 300}, {0, 100},    // E4 D4 E4 rest
  {294, 150}, {330, 150}, {392, 150}, {330, 150},  // D4 E4 G4 E4
  {294, 150}, {262, 150}, {294, 300}, {0, 100}     // D4 C4 D4 rest
};
int snakeMusicNote = 0;
unsigned long snakeMusicLastNote = 0;
bool snakeMusicPlaying = false;

// Paint app state
#define PAINT_WIDTH 50
#define PAINT_HEIGHT 28
uint16_t paintCanvas[PAINT_HEIGHT][PAINT_WIDTH];  // Canvas pixels (RGB565)
int paintCursorX, paintCursorY;
uint16_t paintColor;
int paintColorIdx;
bool paintSaved;
bool paintEraser = false;  // true = eraser mode, false = brush mode
const uint16_t paintPalette[10] = {0x0000, 0xFFFF, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFDA0, 0x8410};  // Black, White, Red, Green, Blue, Yellow, Magenta, Cyan, Orange, Gray

// Paint session tracking - for save functionality
int paintSessionNum = 0;        // Current session number (increments on new canvas)
bool paintSessionSaved = false; // Has current session been saved at least once?
char paintSessionFile[64] = ""; // Current session filename

// BMP viewer state (opened from Files app)
bool viewingBMP = false;
char bmpViewPath[MAX_PATH_LEN] = "";
// Separate persistent buffer for BMP viewing (allocated once, reused)
uint16_t* bmpViewBuffer = nullptr;
int bmpViewWidth = 0, bmpViewHeight = 0;
// Image viewer overlay state
unsigned long imgViewerOverlayTime = 0;  // Time when overlay was last shown
bool imgViewerOverlayVisible = true;     // Whether close button overlay is visible
int imgViewerLastCursorX = -1, imgViewerLastCursorY = -1;  // Last cursor position

// Screenshot counter and delayed refresh
int screenshotNum = 0;
unsigned long screenshotTakenTime = 0;  // Time when last screenshot was taken (for delayed refresh)

// SD card warning counter - show popup after 3 consecutive warnings
int sdCardWarningCount = 0;

// Delete confirmation state
bool deleteConfirmMode = false;

// Desktop notification popup state
bool notificationPopup = false;
char notificationMsg[32] = "";

// Desktop shortcut double-click detection
unsigned long lastDesktopClick = 0;
int16_t lastClickX = 0, lastClickY = 0;

// ===== DESKTOP MODE STATE =====
bool desktopMode = false;  // true when in desktop mode
bool bootToDesktop = false;  // Setting: boot directly to desktop mode

// Deferred save system (avoids slow SD writes on every action)
bool dataDirty = false;  // Set true when data needs saving
unsigned long lastSaveTime = 0;
#define SAVE_INTERVAL 30000  // Save dirty data every 30 seconds max

// Mouse pointer
int16_t mouseX = 120, mouseY = 67;  // Center of screen
int16_t prevMouseX = 120, prevMouseY = 67;
uint8_t mouseSpeed = 1;  // Accelerates when held
unsigned long mouseHoldStart = 0;

// Desktop UI constants - Platinum/Win98 Theme
#define DESK_TASKBAR_H 20
#define DESK_TITLEBAR_H 18
#define DESK_BTN_SIZE 16
#define DESK_START_W 44
#define DESK_DESKTOP_COLOR 0x0410  // Teal desktop background (Win98)
#define DESK_TASKBAR_COLOR 0xC618  // Silver taskbar
#define DESK_WINDOW_COLOR 0xC618   // Window content bg (silver)
#define DESK_TITLEBAR_COLOR 0x0010 // Navy blue active titlebar
#define DESK_TITLEBAR_INACTIVE 0x8410 // Gray inactive titlebar
#define DESK_TITLEBAR_TEXT 0xFFFF  // White text on titlebar
#define DESK_BTN_FACE 0xC618       // Button face color
#define DESK_BTN_LIGHT 0xFFFF      // Button highlight
#define DESK_BTN_DARK 0x8410       // Button shadow

// Taskbar auto-hide state
bool taskbarHidden = false;

// Start menu - all apps except Desktop, plus Sleep, Restart, Exit
bool startMenuOpen = false;
int startMenuSel = 0;
#define START_MENU_ITEMS 14
// Alphabetical order: Calc, Editor, Files, Music, Notes, Paint, Settings, Snake, Tetris, Timer, TODO, then Sleep, Restart, Exit
const char* startMenuItems[START_MENU_ITEMS] = {"Calc", "Editor", "Files", "Music", "Notes", "Paint", "Settings", "Snake", "Tetris", "Timer", "TODO", "Sleep", "Restart", "Exit"};

// Sleep mode state
bool sleepMode = false;
unsigned long sleepStartTime = 0;  // When sleep started (for 1s delay before wake)

// Desktop windows (increased limit)
#define MAX_DESK_WINDOWS 15
struct DeskWindow {
  bool open;
  int16_t x, y, w, h;
  int16_t minW, minH;
  uint8_t appType;  // 0=notepad, 1=calc, 2=about
  bool maximized;
  bool fullscreen;  // True fullscreen (no title bar, covers taskbar)
  char title[12];
};
DeskWindow deskWins[MAX_DESK_WINDOWS];
int8_t deskWinOrder[MAX_DESK_WINDOWS];  // Z-order (index 0 = top)
int8_t deskWinCount = 0;
int8_t activeWin = -1;  // Currently focused window

// Dragging/resizing state
bool dragging = false;
bool resizing = false;
int16_t dragOffX, dragOffY;
int8_t dragWinIdx = -1;

// Click state
bool mouseDown = false;
unsigned long mouseDownTime = 0;

// Note frequencies
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_DS4 311
#define NOTE_REST 0

// Forward declarations
void drawHomeScreen();
void drawAppDrawer(bool fullRedraw);
void drawMenuBar();
void drawAppIcon(int x, int y, int appIndex);
void drawAppRow(int appIndex, bool selected);
void drawAppRowBuffered(Adafruit_GFX* g, int appIndex, bool selected);
void drawAppIconBuffered(Adafruit_GFX* g, int x, int y, int appIndex);
void updateTime();
void updateHomeTime();
void updateMenuBarTime();
void runApp();
void drawSettingsApp();
void drawCalcApp();
void drawTimerApp();
void drawTodoApp();
void drawNotesApp();
void drawMusicApp();
void drawDesktopApp();
void drawAppSwitcher();
void drawNotification();
void showNotification(const char* text);
void tetrisInitGame();
void tetrisSpawnPiece();
bool tetrisCheckCollision(int px, int py, int rot);
void tetrisLockPiece();
void playQuietTone(uint16_t freq, uint8_t volume = 20);  // Forward declaration
void snakeInitGame();
void snakeSpawnFood();
void snakeUpdate();
void paintInitCanvas();
void paintSaveToSD();
void drawSnakeApp();
void drawPaintApp();
void drawTetrisApp();
void tetrisClearLines();
void tetrisMove(int dx);
void tetrisRotate();
void tetrisDrop();
void tetrisHardDrop();
void tetrisUpdate();
void tetrisMusicUpdate();
void snakeMusicUpdate();
void dismissNotification();
void updateBackgroundTimer();
void ensureOSDataFolder();
void loadWiFiCredentials();
void saveWiFiCredentials();
void connectWiFi(bool drawProgress = true);
void syncTimeFromNTP();
void scanWiFiNetworks();
void drawWiFiEditor();

void setup() {
  Serial.begin(115200);
  Serial.println("arduinOS Starting on ESP32...");

  // Initialize joystick
  pinMode(JOY_UP, INPUT_PULLUP);
  pinMode(JOY_DOWN, INPUT_PULLUP);
  pinMode(JOY_LEFT, INPUT_PULLUP);
  pinMode(JOY_RIGHT, INPUT_PULLUP);
  pinMode(JOY_MID, INPUT_PULLUP);
  pinMode(JOY_SET, INPUT_PULLUP);
  pinMode(JOY_RST, INPUT_PULLUP);
  
  // Initialize backlight pin
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);  // Backlight ON by default

  // Initialize buzzer PWM (ESP32 LEDC - new API for ESP32 core 3.x)
  ledcAttach(BUZZER_PIN, 5000, 8);
  ledcWriteTone(BUZZER_PIN, 0);  // Ensure buzzer is off at boot
  ledcWrite(BUZZER_PIN, 0);
  
  // Initialize hardware timer for non-blocking sound
  initSoundTimer();

  // Initialize display with stable SPI speed
  delay(100);  // Give TFT time to stabilize after upload/reset
  tft.init(135, 240, SPI_MODE3);
  tft.setRotation(3);
  tft.setSPISpeed(20000000);  // 20MHz for stability (was 27MHz, originally 40MHz)
  delay(50);  // Additional delay for display to be ready
  tft.fillScreen(COLOR_BG);
  
  // Initialize TFT refresh timer AFTER display is ready
  lastTFTReinit = millis();
  
  // Allocate double buffer early for consistent rendering
  initDoubleBuffer();
  
  // Load BIOS settings from ESP32 NVS (before SD card check)
  loadBiosSettings();
  
  // Check for BIOS entry - hold SET within first 3 seconds for 1 second
  unsigned long bootStart = millis();
  unsigned long setHoldStart = 0;
  bool biosEntered = false;
  
  // Show ArduinOS logo with BIOS hint
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_ACCENT);
  tft.setTextSize(3);
  const char* biosTitle = "ArduinOS";
  int16_t biosTitleW = strlen(biosTitle) * 6 * 3;
  tft.setCursor((SCREEN_WIDTH - biosTitleW) / 2, 35);
  tft.print(biosTitle);
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF);
  tft.setCursor(55, 75);
  tft.print("Hold SET for BIOS...");
  
  while (millis() - bootStart < 3000 && !biosEntered) {
    if (digitalRead(JOY_SET) == LOW) {
      if (setHoldStart == 0) setHoldStart = millis();
      if (millis() - setHoldStart >= 1000) {
        // Enter BIOS
        biosEntered = true;
        showBiosScreen();
      }
    } else {
      setHoldStart = 0;
    }
    delay(10);
  }
  
  tft.fillScreen(COLOR_BG);  // Clear BIOS hint

  // Initialize SD card on HSPI
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);  // Deselect SD card first
  
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  delay(50);
  
  // Quick first attempt
  if (SD.begin(SD_CS, hspi)) {
    sdCardMounted = true;
    Serial.println("SD card mounted on first try");
  } else {
    // If first attempt fails, try a few more times with brief delays
    for (int attempt = 1; attempt < 4 && !sdCardMounted; attempt++) {
      SD.end();
      delay(100);
      if (SD.begin(SD_CS, hspi)) {
        sdCardMounted = true;
        Serial.print("SD card mounted on attempt ");
        Serial.println(attempt + 1);
      }
    }
  }
  
  if (!sdCardMounted) {
    Serial.println("SD card not detected at boot");
  }
  
  // Require SD card on boot (if BIOS setting enabled) - wait until inserted
  // Use double buffer for consistent rendering
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  while (biosRequireSD && !sdCardMounted) {
    g->fillScreen(0x0000);  // Black background
    // Draw popup message - original dark theme style
    int popW = 180, popH = 70;
    int popX = (SCREEN_WIDTH - popW) / 2;
    int popY = (SCREEN_HEIGHT - popH) / 2;
    g->fillRoundRect(popX, popY, popW, popH, 8, 0x4208);  // Dark gray bg
    g->drawRoundRect(popX, popY, popW, popH, 8, COLOR_ACCENT);
    g->setTextSize(1);
    g->setTextColor(0xF800);  // Red
    g->setCursor(popX + 30, popY + 12);
    g->print("No SD Card Found!");
    g->setTextColor(COLOR_TEXT);
    g->setCursor(popX + 15, popY + 28);
    g->print("Please insert SD card");
    g->setCursor(popX + 35, popY + 42);
    g->print("to continue...");
    g->setTextColor(0x7BEF);
    g->setCursor(popX + 25, popY + 56);
    g->print("Checking every 1s");
    
    // Flush buffer to screen
    flushBuffer();
    
    delay(1000);
    // Try to mount again with fresh SPI init
    SD.end();
    hspi.end();
    delay(200);
    hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    delay(200);
    if (SD.begin(SD_CS, hspi)) {
      sdCardMounted = true;
      Serial.println("SD card mounted successfully");
    }
  }

  // Load saved WiFi credentials before drawing boot screen
  loadWiFiCredentials();
  
  // Load saved app data from SD card .osdata folder
  loadOSData();

  // Beautified boot screen
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_ACCENT);
  tft.setTextSize(4);
  const char* bootTitle = "ArduinOS";
  int16_t titleW = strlen(bootTitle) * 6 * 4;
  int16_t titleH = 8 * 4;
  int16_t titleX = (SCREEN_WIDTH - titleW) / 2;
  int16_t titleY = (SCREEN_HEIGHT / 2) - (titleH / 2) - 10;
  tft.setCursor(titleX, titleY);
  tft.print(bootTitle);

  if (biosEnableWiFi && strlen(wifiSSID) > 0) {
    tft.setTextColor(0x6B6D);
    tft.setTextSize(2);
    const char* bootConn = "Connecting...";
    int16_t connW = strlen(bootConn) * 6 * 2;
    int16_t connX = (SCREEN_WIDTH - connW) / 2;
    int16_t connY = SCREEN_HEIGHT - 28;
    tft.setCursor(connX, connY);
    tft.print(bootConn);

    connectWiFi(false);
  }

  tft.fillScreen(COLOR_BG);

  // Calculate visible apps
  maxVisibleApps = (SCREEN_HEIGHT - MENUBAR_HEIGHT) / APP_ROW_HEIGHT;

  lastTime = millis();
  lastSecond = millis();
  
  // Initialize game states
  snakeShowMenu = true;
  snakePlaying = false;
  snakeGameOver = false;
  snakePaused = false;
  tetrisShowMenu = true;
  tetrisPlaying = false;
  tetrisGameOver = false;
  tetrisPaused = false;

  // Double buffer will be allocated lazily when entering desktop/game mode
  // This avoids memory issues during boot

  // Draw initial screen - check if boot to desktop is enabled
  if (bootToDesktop) {
    // Boot directly to desktop mode
    currentScreen = SCREEN_APP;
    selectedApp = APP_DESKTOP;
    drawDesktopApp();  // Initialize desktop mode properly
  } else {
    drawHomeScreen();
  }
}

// Cached joystick state - read once per frame to prevent GPIO overload
bool joyUp = false, joyDown = false, joyLeft = false, joyRight = false;
bool joyMid = false, joySet = false, joyRst = false;

void readJoystick() {
  // Read all joystick pins once per frame
  joyUp = (digitalRead(JOY_UP) == LOW);
  joyDown = (digitalRead(JOY_DOWN) == LOW);
  joyLeft = (digitalRead(JOY_LEFT) == LOW);
  joyRight = (digitalRead(JOY_RIGHT) == LOW);
  joyMid = (digitalRead(JOY_MID) == LOW);
  joySet = (digitalRead(JOY_SET) == LOW);
  joyRst = (digitalRead(JOY_RST) == LOW);
}

void loop() {
  unsigned long currentTime = millis();
  lastTime = currentTime;
  
  // Frame rate limiter - prevent loop from running faster than 30fps (~33ms per frame)
  // This prevents overwhelming the system with too many GPIO reads and redraws
  static unsigned long lastFrame = 0;
  if (currentTime - lastFrame < 33) {
    yield();
    return;  // Skip this frame
  }
  lastFrame = currentTime;
  
  // Watchdog feed - critical to prevent system crash
  yield();
  esp_task_wdt_reset();
  
  // Periodic TFT reinit - keeps display alive, runs in background
  periodicTFTReinit();
  
  // Periodic heap check and debug output
  static unsigned long lastHeapCheck = 0;
  if (currentTime - lastHeapCheck >= 5000) {
    lastHeapCheck = currentTime;
    uint32_t freeHeap = ESP.getFreeHeap();
    Serial.print("Heap: ");
    Serial.print(freeHeap);
    Serial.print(" | Uptime: ");
    Serial.print(currentTime / 1000);
    Serial.println("s");
  }
  
  // Read joystick state once per frame
  readJoystick();
  
  // Yield to prevent watchdog timeout during rapid operations
  yield();
  
  // Update non-blocking sound system
  updateSound();
  
  // Shared RST state to prevent double-triggering between game apps and main handler
  static bool rstWaitForRelease = false;  // If true, wait for RST to be released before processing again

  // SD card hot-swap detection (check every 2 seconds)
  static unsigned long lastSDCheck = 0;
  if (sdCardMounted && !sdCardRemoved && currentTime - lastSDCheck >= 2000) {
    lastSDCheck = currentTime;
    if (!checkSDCard()) {
      // SD card check failed - increment warning counter
      sdCardWarningCount++;
      Serial.print("SD card check failed, warning count: ");
      Serial.println(sdCardWarningCount);
      showNotification("SD Warning!");
      
      // After 2 consecutive warnings (4 seconds), treat as removed
      if (sdCardWarningCount >= 2) {
        sdCardMounted = false;
        sdCardWarningCount = 0;  // Reset counter
        // Only show popup and block if biosRequireSD is enabled
        if (biosRequireSD) {
          sdCardRemoved = true;
          // Switch to desktop mode with minimal UI
          desktopMode = true;
          currentScreen = SCREEN_APP;
          selectedApp = APP_DESKTOP;
          // Close all windows
          for (int i = 0; i < MAX_DESK_WINDOWS; i++) deskWins[i].open = false;
          deskWinCount = 0;
          activeWin = -1;
          playBack();  // Short beep instead of long error sound
        }
        // If biosRequireSD is false, just mark as unmounted but continue normally
      }
    } else {
      // SD card check passed - reset warning counter
      sdCardWarningCount = 0;
    }
  }
  
  // Try to remount SD card periodically if not mounted (every 5 seconds to reduce lag)
  static unsigned long lastRemountTry = 0;
  if (!sdCardMounted && currentTime - lastRemountTry >= 5000) {
    lastRemountTry = currentTime;
    if (remountSDCard()) {
      // Card inserted/reinserted! Reload system
      sdCardRemoved = false;
      loadOSData();
      loadWiFiCredentials();
      showNotification("SD Reloaded!");
      deskNeedRedraw = true;
    }
  }
  
  // SD card removed mode - show popup and wait for reinsertion (only if biosRequireSD)
  if (sdCardRemoved && biosRequireSD) {
    // Use double buffer for consistent rendering
    if (!bufferReady) initDoubleBuffer();
    Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
    
    // Draw minimal desktop with popup - Win98 style
    g->fillScreen(DESK_DESKTOP_COLOR);  // Teal desktop background
    // Draw popup message with 3D raised border
    int popW = 180, popH = 70;
    int popX = (SCREEN_WIDTH - popW) / 2;
    int popY = (SCREEN_HEIGHT - popH) / 2;
    // Popup background
    g->fillRect(popX, popY, popW, popH, COLOR_3D_FACE);
    // 3D raised border
    g->drawFastHLine(popX, popY, popW, COLOR_3D_LIGHT);
    g->drawFastVLine(popX, popY, popH, COLOR_3D_LIGHT);
    g->drawFastHLine(popX, popY + popH - 1, popW, COLOR_3D_DARK);
    g->drawFastVLine(popX + popW - 1, popY, popH, COLOR_3D_DARK);
    g->drawRect(popX, popY, popW, popH, 0x0000);  // Outer black border
    // Title bar
    g->fillRect(popX + 2, popY + 2, popW - 4, 14, DESK_TITLEBAR_COLOR);
    g->setTextSize(1);
    g->setTextColor(0xFFFF);
    g->setCursor(popX + 6, popY + 5);
    g->print("Error");
    // Message
    g->setTextColor(COLOR_TEXT);
    g->setCursor(popX + 20, popY + 24);
    g->print("SD Card Removed!");
    g->setCursor(popX + 12, popY + 38);
    g->print("Please reinsert SD");
    g->setCursor(popX + 12, popY + 50);
    g->print("card to continue...");
    
    // Draw mouse cursor
    g->fillTriangle(mouseX, mouseY, mouseX + 10, mouseY + 5, mouseX + 5, mouseY + 10, 0xFFFF);
    g->drawTriangle(mouseX, mouseY, mouseX + 10, mouseY + 5, mouseX + 5, mouseY + 10, 0x0000);
    
    // Flush buffer to screen
    flushBuffer();
    
    delay(100);  // Slow down loop while waiting
    return;  // Don't process anything else
  }

  // Screenshot: Hold SET + RST together for 1 second to take screenshot
  // Uses external flags setWasPressed/rstWasPressed to prevent release actions
  static unsigned long screenshotHoldStart = 0;
  static bool screenshotTaken = false;  // Prevent multiple screenshots per hold
  static bool screenshotComboUsed = false;  // Prevent SET/RST actions after screenshot combo
  if (joySet && joyRst) {
    if (screenshotHoldStart == 0) {
      screenshotHoldStart = currentTime;
      screenshotTaken = false;
      screenshotComboUsed = true;  // Mark that combo was used
    }
    if (!screenshotTaken && currentTime - screenshotHoldStart >= 1000) {
      takeScreenshot();
      screenshotTaken = true;
    }
    // While both buttons held, don't process individual button actions
    return;
  } else if (screenshotComboUsed && (!joySet || !joyRst)) {
    // One or both buttons released after combo - wait for both to be released
    if (!joySet && !joyRst) {
      screenshotComboUsed = false;
      screenshotHoldStart = 0;
      screenshotTaken = false;
    }
    // Don't process individual button actions while releasing from combo
    return;
  } else {
    screenshotHoldStart = 0;
    screenshotTaken = false;
  }

  // Handle sleep mode - wake on any joystick input after 1s delay
  if (sleepMode) {
    // Only allow wake after 1 second of sleep
    if (currentTime - sleepStartTime >= 1000) {
      bool anyInput = joyUp || joyDown || joyLeft || joyRight || joyMid || joyRst || joySet;
      if (anyInput) {
        sleepMode = false;
        digitalWrite(TFT_BLK, HIGH);  // Turn backlight ON
        deskNeedRedraw = true;  // Trigger desktop redraw
        delay(200);  // Debounce
      }
    }
    return;  // Don't process anything else while sleeping
  }

  // Update time every second
  if (currentTime - lastSecond >= 1000) {
    lastSecond = currentTime;
    updateTime();
    
    if (minutes != lastDisplayedMinute) {
      lastDisplayedMinute = minutes;
      if (currentScreen == SCREEN_HOME) updateHomeTime();
      else if (currentScreen != SCREEN_APP) updateMenuBarTime();
    }
    
    // Periodic TFT refresh every 10 seconds to prevent configuration drift
    static uint8_t spiRefreshCounter = 0;
    spiRefreshCounter++;
    if (spiRefreshCounter >= 10) {
      spiRefreshCounter = 0;
      // Full TFT reinit to prevent any accumulated SPI issues
      tft.setSPISpeed(20000000);
      yield();
    }
  }
  
  // Background timer - always runs
  updateBackgroundTimer();
  
  // Non-blocking WiFi connection check
  if (wifiConnecting) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnecting = false;
      wifiConnected = true;
      showNotification("WiFi Connected!");
      deskNeedRedraw = true;  // Trigger desktop redraw
      // Start non-blocking NTP sync
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      ntpSyncing = true;
      ntpSyncStart = currentTime;
    } else if (currentTime - wifiConnectStart > 10000) {
      // Timeout after 10 seconds
      wifiConnecting = false;
      wifiConnected = false;
      showNotification("WiFi Failed");
      deskNeedRedraw = true;  // Trigger desktop redraw
    }
  }
  
  // Non-blocking NTP sync check
  if (ntpSyncing) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10)) {  // 10ms timeout to prevent blocking
      ntpSyncing = false;
      // Time synced successfully
    } else if (currentTime - ntpSyncStart > 5000) {
      // Timeout after 5 seconds
      ntpSyncing = false;
    }
  }
  
  // Non-blocking WiFi scan check
  if (wifiScanning) {
    int16_t result = WiFi.scanComplete();
    if (result >= 0) {
      wifiScanning = false;
      wifiScanCount = min(result, (int16_t)MAX_WIFI_NETWORKS);
      for (int i = 0; i < wifiScanCount; i++) {
        WiFi.SSID(i).toCharArray(wifiNetworks[i], 33);
      }
      WiFi.scanDelete();  // CRITICAL: Free scan results memory to prevent leak
    } else if (result == WIFI_SCAN_FAILED) {
      wifiScanning = false;
      wifiScanCount = 0;
      WiFi.scanDelete();  // CRITICAL: Free scan results memory even on failure
    }
  }
  
  // Update timer display if viewing timer app
  if (currentScreen == SCREEN_APP && selectedApp == APP_TIMER && (timerOn || timerPaused)) {
    static unsigned long lastTimerDraw = 0;
    if (currentTime - lastTimerDraw >= 100) {
      lastTimerDraw = currentTime;
      lastButton = 0;
      drawTimerApp();
    }
  }
  
  // Use cached joystick state (read once per frame in readJoystick())
  bool midState = !joyMid;  // Convert back to HIGH/LOW for compatibility
  bool upState = !joyUp;
  bool downState = !joyDown;
  bool leftState = !joyLeft;
  bool rightState = !joyRight;
  bool setState = !joySet;
  bool rstState = !joyRst;

  // Desktop mode handles its own input - just call runApp and skip normal input handling
  if (desktopMode && currentScreen == SCREEN_APP && selectedApp == APP_DESKTOP) {
    runApp();
    // Update last states so they're current when exiting desktop mode
    lastUpState = upState;
    lastDownState = downState;
    lastLeftState = leftState;
    lastRightState = rightState;
    lastMidState = midState;
    lastSetState = setState;
    return;
  }
  
  // Non-desktop game apps handle their own input continuously (like desktop mode)
  if (currentScreen == SCREEN_APP && (selectedApp == APP_PAINT || selectedApp == APP_SNAKE || selectedApp == APP_TETRIS)) {
    // Handle SET button with debounce - go back to drawer
    static unsigned long gameSetStart = 0;
    static bool gameSetDebounced = false, gameSetActed = false, gameLastSetState = HIGH;
    if (setState == LOW && gameLastSetState == HIGH) { gameSetStart = currentTime; gameSetDebounced = false; gameSetActed = false; }
    if (setState == LOW && !gameSetDebounced && currentTime - gameSetStart >= 50) gameSetDebounced = true;
    if (setState == HIGH) { gameSetDebounced = false; gameSetActed = false; }
    gameLastSetState = setState;
    
    if (gameSetDebounced && !gameSetActed) {
      gameSetActed = true;
      // Stop any playing music/buzzer
      ledcWrite(BUZZER_PIN, 0);
      snakeMusicPlaying = false;
      tetrisMusicPlaying = false;
      // Reset Paint canvas when exiting Paint app (start fresh next time)
      if (selectedApp == APP_PAINT) {
        paintInitCanvas();
      }
      playBack();
      currentScreen = SCREEN_DRAWER;
      prevSelectedApp = -1;
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      drawAppDrawer(true);
      lastSetState = setState;
      return;
    }
    
    // Handle RST button with debounce - open app switcher
    static unsigned long gameRstStart = 0;
    static bool gameRstDebounced = false, gameRstActed = false, gameLastRstState = HIGH;
    if (rstState == LOW && gameLastRstState == HIGH) { gameRstStart = currentTime; gameRstDebounced = false; gameRstActed = false; }
    if (rstState == LOW && !gameRstDebounced && currentTime - gameRstStart >= 50) gameRstDebounced = true;
    if (rstState == HIGH) { gameRstDebounced = false; gameRstActed = false; }
    gameLastRstState = rstState;
    
    if (gameRstDebounced && !gameRstActed) {
      gameRstActed = true;
      // Stop any playing music/buzzer
      ledcWrite(BUZZER_PIN, 0);
      snakeMusicPlaying = false;
      tetrisMusicPlaying = false;
      currentScreen = SCREEN_SWITCHER;
      switcherSel = selectedApp;
      tft.fillScreen(COLOR_BG);
      drawAppSwitcher();
      lastSetState = setState;
      // Mark RST as needing release before next action
      rstWaitForRelease = true;
      return;
    }
    
    // Handle directional and MID inputs for game apps with hold-to-repeat
    static unsigned long gameHoldStart = 0;      // When current direction started being held
    static unsigned long gameLastRepeat = 0;     // Last time we triggered a repeat
    static int gameHeldDir = 0;                  // Which direction is held (1-4), 0 = none
    static bool gameFirstPress = true;           // Is this the first press (not a repeat)?
    
    // MID button - single press only, no repeat
    static unsigned long gameMidStart = 0;
    static bool gameMidDebounced = false, gameMidActed = false;
    static bool gameLastMid = HIGH;
    
    if (midState == LOW && gameLastMid == HIGH) { gameMidStart = currentTime; gameMidDebounced = false; gameMidActed = false; }
    if (midState == LOW && !gameMidDebounced && currentTime - gameMidStart >= 50) gameMidDebounced = true;
    if (midState == HIGH) { gameMidDebounced = false; gameMidActed = false; }
    gameLastMid = midState;
    
    // Directional buttons with hold-to-repeat (like desktop mode)
    bool upHeld = (upState == LOW);
    bool downHeld = (downState == LOW);
    bool leftHeld = (leftState == LOW);
    bool rightHeld = (rightState == LOW);
    
    // Detect new press
    bool upPressed = (upHeld && lastUpState == HIGH);
    bool downPressed = (downHeld && lastDownState == HIGH);
    bool leftPressed = (leftHeld && lastLeftState == HIGH);
    bool rightPressed = (rightHeld && lastRightState == HIGH);
    
    // Track which direction is held
    if (upPressed) { gameHoldStart = currentTime; gameHeldDir = 1; gameFirstPress = true; }
    else if (downPressed) { gameHoldStart = currentTime; gameHeldDir = 2; gameFirstPress = true; }
    else if (leftPressed) { gameHoldStart = currentTime; gameHeldDir = 3; gameFirstPress = true; }
    else if (rightPressed) { gameHoldStart = currentTime; gameHeldDir = 4; gameFirstPress = true; }
    
    // Reset if no direction held
    if (!upHeld && !downHeld && !leftHeld && !rightHeld) {
      gameHeldDir = 0;
      gameFirstPress = true;
    }
    
    // Determine if we should trigger (first press after 50ms debounce, or repeat after hold delay)
    int repeatDelay = 150;  // ms between repeats
    int holdDelay = 400;    // ms before repeats start
    bool holdLongEnough = (gameHoldStart > 0 && currentTime - gameHoldStart > holdDelay);
    
    bool shouldTrigger = false;
    int triggerDir = 0;
    
    if (gameHeldDir > 0) {
      if (gameFirstPress && currentTime - gameHoldStart >= 50) {
        // First press after debounce
        shouldTrigger = true;
        triggerDir = gameHeldDir;
        gameFirstPress = false;
        gameLastRepeat = currentTime;
      } else if (!gameFirstPress && holdLongEnough && currentTime - gameLastRepeat > repeatDelay) {
        // Repeat while holding
        shouldTrigger = true;
        triggerDir = gameHeldDir;
        gameLastRepeat = currentTime;
      }
    }
    
    // Set lastButton for the app to consume
    if (shouldTrigger && triggerDir > 0) { lastButton = triggerDir; }
    else if (gameMidDebounced && !gameMidActed) { lastButton = 5; gameMidActed = true; }
    
    runApp();
    // Update last states
    lastUpState = upState;
    lastDownState = downState;
    lastLeftState = leftState;
    lastRightState = rightState;
    lastMidState = midState;
    lastSetState = setState;
    return;
  }

  // MID button with hold detection for notification dismiss and repeat
  static unsigned long lastMidAction = 0;  // Time of last MID action (for debounce)
  static unsigned long midDebounceStart = 0;  // When button first went LOW
  static bool midDebounced = false;  // Has debounce period passed?
  static bool midActedThisPress = false;   // Already acted on this button press?
  
  // Hardware debounce - require button to be held LOW for 50ms before registering
  if (midState == LOW && lastMidState == HIGH) {
    midDebounceStart = currentTime;
    midDebounced = false;
  }
  if (midState == LOW && !midDebounced && currentTime - midDebounceStart >= 50) {
    midDebounced = true;
    midPressStart = currentTime;
    midWasHeld = false;
    midActedThisPress = false;
  }
  if (midState == HIGH) {
    midDebounced = false;
  }
  
  // Check for hold to dismiss notification
  if (midState == LOW && midDebounced && notifActive && currentTime - midPressStart > 500) {
    dismissNotification();
    midWasHeld = true;
  }
  
  // MID action - minimum 150ms between any clicks OS-wide
  bool midAction = false;
  if (midState == LOW && midDebounced && !notifActive && !midWasHeld && currentTime - lastMidAction >= 150) {
    if (!midActedThisPress) {
      // First action on this button press
      midAction = true;
      midActedThisPress = true;
      lastMidAction = currentTime;
    } else if (currentTime - midPressStart > 400) {
      // Repeat after 400ms initial delay, then every 100ms
      midAction = true;
      lastMidAction = currentTime;
    }
  }
  
  if (midAction) {
    yield();  // Non-blocking alternative to delay(30)
    if (currentScreen == SCREEN_HOME) {
      playSelect();
      currentScreen = SCREEN_DRAWER;
      prevSelectedApp = -1;
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      drawAppDrawer(true);
    } else if (currentScreen == SCREEN_DRAWER) {
      playSelect();
      currentScreen = SCREEN_APP;
      appOpen[selectedApp] = true;  // Mark app as open
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      lastButton = 0;
      runApp();
    } else if (currentScreen == SCREEN_SWITCHER) {
      playSelect();
      selectedApp = switcherSel;
      currentScreen = SCREEN_APP;
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      lastButton = 0;
      runApp();
    } else if (currentScreen == SCREEN_APP) {
      playTick();
      lastButton = 5;
      runApp();
    }
  }
  lastMidState = midState;

  // SET button - go back (app stays open in background)
  // Trigger on RELEASE (HIGH after LOW) to allow SET+RST screenshot without side effects
  static bool setWasPressed = false;
  if (setState == LOW) {
    setWasPressed = true;
  }
  if (setState == HIGH && lastSetState == LOW && setWasPressed) {
    setWasPressed = false;
    yield();
    playBack();
    // Handle TODO editor SET - go back to list
    if (todoEditMode && currentScreen == SCREEN_APP && selectedApp == APP_TODO) {
      todoEditMode = false;
      lastButton = 0;
      dataDirty = true;  // Mark data as changed
      // Force redraw of list mode (drawTodoApp uses buffer now)
      drawTodoApp();
      lastSetState = setState;
      return;
    }
    // Handle WiFi editor SET
    else if (wifiEditMode) {
      if (wifiScanMode) {
        // Rescan networks
        scanWiFiNetworks();
        lastButton = 0;
        drawWiFiEditor();
      } else {
        // Go back to network selection
        wifiScanMode = true;
        lastButton = 0;
        drawWiFiEditor();
      }
    } else if (currentScreen == SCREEN_APP) {
      // Don't close app, just hide it (timer keeps running)
      currentScreen = SCREEN_DRAWER;
      prevSelectedApp = -1;
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      drawAppDrawer(true);
    } else if (currentScreen == SCREEN_DRAWER) {
      currentScreen = SCREEN_HOME;
      lastDisplayedMinute = -1;
      drawHomeScreen();
    } else if (currentScreen == SCREEN_SWITCHER) {
      currentScreen = SCREEN_HOME;
      lastDisplayedMinute = -1;
      drawHomeScreen();
    }
  }
  lastSetState = setState;
  
  // RST button - open app switcher or close selected app
  // Trigger on RELEASE (HIGH after LOW) to allow SET+RST screenshot without side effects
  static bool lastRstState = HIGH;
  static bool rstWasPressed = false;
  // Track RST press
  if (rstState == LOW) {
    rstWasPressed = true;
  }
  // Clear wait-for-release flag when RST is released
  if (rstState == HIGH && rstWaitForRelease) {
    rstWaitForRelease = false;
    rstWasPressed = false;  // Don't trigger action after wait-for-release
  }
  // Skip if waiting for RST release (prevent double-trigger from game apps)
  if (rstWaitForRelease) {
    lastRstState = rstState;
  } else if (rstState == HIGH && lastRstState == LOW && rstWasPressed) {
    rstWasPressed = false;
    yield();
    // Handle TODO editor RST - save and exit app
    if (todoEditMode && currentScreen == SCREEN_APP && selectedApp == APP_TODO) {
      todoEditMode = false;
      dataDirty = true;  // Mark data as changed
      playBack();
      // Exit to drawer
      currentScreen = SCREEN_DRAWER;
      prevSelectedApp = -1;
      tft.fillScreen(COLOR_BG);
      drawMenuBar();
      drawAppDrawer(true);
      lastRstState = rstState;
      return;
    }
    // Handle WiFi editor RST
    if (wifiEditMode) {
      if (wifiScanMode) {
        // Exit WiFi editor without connecting
        wifiEditMode = false;
        wifiScanMode = false;
        lastButton = 0;
        drawSettingsApp();
      } else {
        // Save & connect
        wifiEditMode = false;
        wifiScanMode = false;
        saveWiFiCredentials();
        // Show connecting message using buffer
        if (!bufferReady) initDoubleBuffer();
        Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
        g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
        g->setTextSize(2);
        g->setTextColor(COLOR_TEXT);
        g->setCursor(40, 60);
        g->print("Connecting...");
        flushBuffer();
        connectWiFi();
        if (wifiConnected) {
          showNotification("WiFi Connected!");
        } else {
          showNotification("WiFi Failed");
        }
        lastButton = 0;
        drawSettingsApp();
      }
    } else if (currentScreen != SCREEN_SWITCHER) {
      // Open switcher
      currentScreen = SCREEN_SWITCHER;
      switcherSel = selectedApp;
      tft.fillScreen(COLOR_BG);
      drawAppSwitcher();
    } else {
      // Close the selected app in switcher
      if (appOpen[switcherSel]) {
        appOpen[switcherSel] = false;
        // If closing timer, stop it
        if (switcherSel == APP_TIMER) {
          timerOn = false;
          timerPaused = false;
        }
        // Reset game states to main menu when closing
        if (switcherSel == APP_SNAKE) {
          snakeShowMenu = true;
          snakePlaying = false;
          snakeGameOver = false;
          snakePaused = false;
          ledcWrite(BUZZER_PIN, 0);
          snakeMusicPlaying = false;
        }
        if (switcherSel == APP_TETRIS) {
          tetrisShowMenu = true;
          tetrisPlaying = false;
          tetrisGameOver = false;
          tetrisPaused = false;
          ledcWrite(BUZZER_PIN, 0);
          tetrisMusicPlaying = false;
        }
        if (switcherSel == APP_PAINT) {
          // Reset paint canvas and start new session
          paintInitCanvas();
        }
        // Find next open app or close switcher
        bool foundOpen = false;
        for (int i = 0; i < NUM_APPS; i++) {
          if (appOpen[i] || (i == APP_TIMER && (timerOn || timerPaused))) {
            switcherSel = i;
            foundOpen = true;
            break;
          }
        }
        if (foundOpen) {
          drawAppSwitcher();
        } else {
          // No more open apps, go home
          currentScreen = SCREEN_HOME;
          lastDisplayedMinute = -1;
          drawHomeScreen();
        }
      }
    }
  }
  lastRstState = rstState;
  
  // Left/Right for app navigation
  if (currentScreen == SCREEN_APP) {
    if (leftState == LOW && lastLeftState == HIGH) {
      yield();  // Non-blocking
      playTick();
      lastButton = 3;
      runApp();
    }
    if (rightState == LOW && lastRightState == HIGH) {
      yield();  // Non-blocking
      playTick();
      lastButton = 4;
      runApp();
    }
  }
  lastLeftState = leftState;
  lastRightState = rightState;

  // Up/Down navigation with hold-to-repeat and proper debouncing
  static unsigned long lastRepeat = 0;
  static unsigned long holdStart = 0;
  static unsigned long upDebounceStart = 0;
  static unsigned long downDebounceStart = 0;
  static bool upDebounced = false;
  static bool downDebounced = false;
  static bool upActed = false;
  static bool downActed = false;
  
  bool upHeld = (upState == LOW);
  bool downHeld = (downState == LOW);
  
  // Hardware debounce for UP - require 50ms stable LOW before registering
  if (upHeld && lastUpState == HIGH) {
    upDebounceStart = currentTime;
    upDebounced = false;
    upActed = false;
  }
  if (upHeld && !upDebounced && currentTime - upDebounceStart >= 50) {
    upDebounced = true;
    holdStart = currentTime;
  }
  if (!upHeld) {
    upDebounced = false;
    upActed = false;
  }
  
  // Hardware debounce for DOWN - require 50ms stable LOW before registering
  if (downHeld && lastDownState == HIGH) {
    downDebounceStart = currentTime;
    downDebounced = false;
    downActed = false;
  }
  if (downHeld && !downDebounced && currentTime - downDebounceStart >= 50) {
    downDebounced = true;
    holdStart = currentTime;
  }
  if (!downHeld) {
    downDebounced = false;
    downActed = false;
  }
  
  if (!upHeld && !downHeld) { holdStart = 0; }
  
  int repeatDelay = 150;
  if (currentScreen == SCREEN_APP && (selectedApp == APP_NOTES || selectedApp == APP_TIMER)) repeatDelay = 80;
  
  bool holdLongEnough = (holdStart > 0 && currentTime - holdStart > 400);
  bool shouldTriggerUp = (upDebounced && !upActed) || (upHeld && upDebounced && holdLongEnough && currentTime - lastRepeat > repeatDelay);
  bool shouldTriggerDown = (downDebounced && !downActed) || (downHeld && downDebounced && holdLongEnough && currentTime - lastRepeat > repeatDelay);
  
  if (shouldTriggerUp || shouldTriggerDown) {
    if (upDebounced && !upActed) upActed = true;
    if (downDebounced && !downActed) downActed = true;
    lastRepeat = currentTime;
    
    if (shouldTriggerUp) {
      if (currentScreen == SCREEN_DRAWER && selectedApp > 0) {
        playTick();
        int oldScroll = scrollOffset;
        selectedApp--;
        if (selectedApp < scrollOffset) scrollOffset = selectedApp;
        drawAppDrawer(scrollOffset != oldScroll);
      } else if (currentScreen == SCREEN_SWITCHER) {
        // Find prev open app
        for (int i = switcherSel - 1; i >= 0; i--) {
          if (appOpen[i] || (i == APP_TIMER && (timerOn || timerPaused))) {
            playTick();
            switcherSel = i;
            drawAppSwitcher();
            break;
          }
        }
      } else if (currentScreen == SCREEN_APP) {
        playTick();
        lastButton = 1;
        runApp();
      }
    }
    if (shouldTriggerDown) {
      if (currentScreen == SCREEN_DRAWER && selectedApp < NUM_APPS - 1) {
        playTick();
        int oldScroll = scrollOffset;
        selectedApp++;
        if (selectedApp >= scrollOffset + maxVisibleApps) scrollOffset = selectedApp - maxVisibleApps + 1;
        drawAppDrawer(scrollOffset != oldScroll);
      } else if (currentScreen == SCREEN_SWITCHER) {
        // Find next open app
        for (int i = switcherSel + 1; i < NUM_APPS; i++) {
          if (appOpen[i] || (i == APP_TIMER && (timerOn || timerPaused))) {
            playTick();
            switcherSel = i;
            drawAppSwitcher();
            break;
          }
        }
      } else if (currentScreen == SCREEN_APP) {
        playTick();
        lastButton = 2;
        runApp();
      }
    }
  }
  lastUpState = upState;
  lastDownState = downState;
}

void updateTime() {
  if (wifiConnected) {
    // Get time from NTP with short timeout to prevent blocking
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10)) {  // 10ms timeout max
      hours = timeinfo.tm_hour;
      minutes = timeinfo.tm_min;
      seconds = timeinfo.tm_sec;
      day = timeinfo.tm_mday;
      month = timeinfo.tm_mon + 1;
      year = timeinfo.tm_year + 1900;
      return;
    }
  }
  // Fallback: manual time counting
  seconds++;
  if (seconds >= 60) {
    seconds = 0;
    minutes++;
    if (minutes >= 60) {
      minutes = 0;
      hours++;
      if (hours >= 24) {
        hours = 0;
      }
    }
  }
}

void drawMenuBar() {
  // Use buffer for consistent rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  // Original dark theme menubar
  g->fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, COLOR_MENUBAR);
  
  g->setTextColor(COLOR_TEXT);
  g->setTextSize(2);
  g->setCursor(4, 1);
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", hours, minutes);
  g->print(timeStr);
  
  // Draw date on right
  g->setCursor(SCREEN_WIDTH - 65, 1);
  char dateStr[6];
  sprintf(dateStr, "%02d/%02d", month, day);
  g->print(dateStr);
  
  // Flush just the menu bar region
  if (bufferReady) {
    tft.drawRGBBitmap(0, 0, deskBuffer->getBuffer(), SCREEN_WIDTH, MENUBAR_HEIGHT);
  }
}

// Fast time-only update for menu bar
void updateMenuBarTime() {
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  g->fillRect(4, 1, 62, 16, COLOR_MENUBAR);
  g->setTextColor(COLOR_TEXT);
  g->setTextSize(2);
  g->setCursor(4, 1);
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", hours, minutes);
  g->print(timeStr);
  
  // Flush just the menu bar region
  if (bufferReady) {
    tft.drawRGBBitmap(0, 0, deskBuffer->getBuffer(), SCREEN_WIDTH, MENUBAR_HEIGHT);
  }
}

void drawHomeScreen() {
  // Use double buffer for consistent rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  g->fillScreen(COLOR_BG);
  
  // Large time in center
  g->setTextColor(COLOR_ACCENT);
  g->setTextSize(5);
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", hours, minutes);
  int16_t timeWidth = 5 * 6 * 5;  // 5 chars * 6 pixels * size 5
  g->setCursor((SCREEN_WIDTH - timeWidth) / 2, 25);
  g->print(timeStr);
  
  // Date below
  g->setTextColor(COLOR_TEXT);
  g->setTextSize(2);
  char dateStr[12];
  sprintf(dateStr, "%02d/%02d/%04d", month, day, year);
  int16_t dateWidth = 10 * 6 * 2;
  g->setCursor((SCREEN_WIDTH - dateWidth) / 2, 75);
  g->print(dateStr);
  
  // Hint at bottom
  g->setTextColor(0x6B6D);
  g->setTextSize(2);
  const char* hint = "MID = Apps";
  int16_t hintWidth = 10 * 6 * 2;
  g->setCursor((SCREEN_WIDTH - hintWidth) / 2, 110);
  g->print(hint);
  
  // Flush buffer to screen
  flushBuffer();
  
  // Redraw notification on top if active (directly to tft)
  if (notifActive) drawNotification();
}

// Fast time-only update for home screen - just redraw the whole screen
void updateHomeTime() {
  drawHomeScreen();  // Use full buffered redraw for consistency
}

// Draw a single app row
void drawAppRow(int appIndex, bool selected) {
  int rowPos = appIndex - scrollOffset;
  if (rowPos < 0 || rowPos >= maxVisibleApps) return;
  
  int y = MENUBAR_HEIGHT + 2 + rowPos * APP_ROW_HEIGHT;
  
  // Draw row background - original dark theme
  tft.fillRect(0, y, SCREEN_WIDTH - 4, APP_ROW_HEIGHT, selected ? COLOR_HIGHLIGHT : COLOR_BG);
  
  // Draw app icon
  drawAppIcon(APP_PADDING, y + 4, appIndex);
  
  // Draw app name
  tft.setTextColor(selected ? COLOR_ACCENT : COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(APP_PADDING + APP_ICON_SIZE + 10, y + 8);
  tft.print(appNames[appIndex]);
}

// Draw app icon with simple graphics
void drawAppIcon(int x, int y, int appIndex) {
  uint16_t color = appColors[appIndex];
  
  // Desktop gets gray background instead of blue
  if (appIndex == APP_DESKTOP) {
    color = 0x4208;  // Dark gray
  }
  
  // Rounded rectangle background
  tft.fillRoundRect(x, y, APP_ICON_SIZE, APP_ICON_SIZE, 4, color);
  
  // Draw simple icon shapes based on app
  uint16_t iconColor = COLOR_TEXT;
  int cx = x + APP_ICON_SIZE / 2;
  int cy = y + APP_ICON_SIZE / 2;
  
  switch (appIndex) {
    case APP_SETTINGS:  // Gear
      tft.drawCircle(cx, cy, 7, iconColor);
      tft.fillCircle(cx, cy, 3, iconColor);
      break;
    case APP_CALC:  // Grid
      tft.drawRect(x + 4, y + 4, 16, 16, iconColor);
      tft.drawLine(x + 4, y + 10, x + 19, y + 10, iconColor);
      tft.drawLine(x + 10, y + 4, x + 10, y + 19, iconColor);
      break;
    case APP_TIMER:  // Clock
      tft.drawCircle(cx, cy, 9, iconColor);
      tft.drawLine(cx, cy, cx, y + 5, iconColor);
      tft.drawLine(cx, cy, x + 17, cy, iconColor);
      break;
    case APP_TODO:  // Checkbox
      tft.drawRect(x + 4, y + 4, 16, 16, iconColor);
      tft.drawLine(x + 7, y + 12, x + 10, y + 16, iconColor);
      tft.drawLine(x + 10, y + 16, x + 17, y + 7, iconColor);
      break;
    case APP_NOTES:  // Lines
      tft.drawRect(x + 4, y + 3, 16, 18, iconColor);
      tft.drawLine(x + 7, y + 7, x + 17, y + 7, iconColor);
      tft.drawLine(x + 7, y + 11, x + 17, y + 11, iconColor);
      tft.drawLine(x + 7, y + 15, x + 14, y + 15, iconColor);
      break;
    case APP_MUSIC:  // Note
      tft.fillCircle(x + 8, y + 16, 4, iconColor);
      tft.fillRect(x + 11, y + 5, 2, 12, iconColor);
      tft.fillRect(x + 11, y + 5, 6, 2, iconColor);
      break;
    case APP_DESKTOP:  // PC icon (gray)
      // Monitor
      tft.fillRect(x + 4, y + 3, 16, 11, 0x6B6D);  // Gray screen
      tft.drawRect(x + 4, y + 3, 16, 11, iconColor);
      // Stand
      tft.fillRect(x + 10, y + 14, 4, 3, 0x6B6D);
      // Base
      tft.fillRect(x + 6, y + 17, 12, 2, 0x6B6D);
      tft.drawRect(x + 6, y + 17, 12, 2, iconColor);
      return;  // Skip the default background
    case APP_FILES:  // Folder icon
      // Folder tab
      tft.fillRect(x + 4, y + 5, 8, 3, 0xFDA0);  // Orange tab
      // Folder body
      tft.fillRect(x + 3, y + 7, 18, 12, 0xFFE0);  // Yellow folder
      tft.drawRect(x + 3, y + 7, 18, 12, iconColor);
      // Folder lines
      tft.drawLine(x + 5, y + 11, x + 18, y + 11, 0xFDA0);
      tft.drawLine(x + 5, y + 14, x + 18, y + 14, 0xFDA0);
      break;
    case APP_PAINT:  // Paintbrush
      // Brush handle
      tft.fillRect(x + 14, y + 4, 4, 10, 0x8410);  // Gray handle
      // Brush tip
      tft.fillRect(x + 6, y + 12, 10, 6, 0xFDA0);  // Orange bristles
      tft.drawRect(x + 6, y + 12, 10, 6, iconColor);
      // Paint drip
      tft.fillCircle(x + 8, y + 19, 2, 0xF800);  // Red paint
      break;
    case APP_SNAKE:  // Snake
      // Snake body (S-shape)
      tft.fillRect(x + 4, y + 6, 6, 4, 0x07E0);   // Green segment
      tft.fillRect(x + 8, y + 9, 4, 6, 0x07E0);
      tft.fillRect(x + 10, y + 13, 6, 4, 0x07E0);
      // Snake head
      tft.fillRect(x + 14, y + 13, 5, 4, 0x07E0);
      tft.fillRect(x + 17, y + 14, 2, 2, 0x0000);  // Eye
      // Food
      tft.fillRect(x + 5, y + 14, 4, 4, 0xF800);  // Red apple
      break;
    case APP_TETRIS:  // Tetris blocks
      // L-piece
      tft.fillRect(x + 4, y + 4, 5, 5, 0xFDA0);   // Orange
      tft.fillRect(x + 4, y + 9, 5, 5, 0xFDA0);
      tft.fillRect(x + 9, y + 9, 5, 5, 0xFDA0);
      // Square piece
      tft.fillRect(x + 14, y + 4, 5, 5, 0xFFE0);  // Yellow
      tft.fillRect(x + 14, y + 9, 5, 5, 0xFFE0);
      // I-piece (partial)
      tft.fillRect(x + 4, y + 14, 5, 5, 0x07FF);  // Cyan
      tft.fillRect(x + 9, y + 14, 5, 5, 0x07FF);
      tft.fillRect(x + 14, y + 14, 5, 5, 0x07FF);
      break;
  }
}

// Buffered version of drawAppRow for double-buffered rendering
void drawAppRowBuffered(Adafruit_GFX* g, int appIndex, bool selected) {
  int rowPos = appIndex - scrollOffset;
  if (rowPos < 0 || rowPos >= maxVisibleApps) return;
  
  int y = MENUBAR_HEIGHT + 2 + rowPos * APP_ROW_HEIGHT;
  
  g->fillRect(0, y, SCREEN_WIDTH - 4, APP_ROW_HEIGHT, selected ? COLOR_HIGHLIGHT : COLOR_BG);
  drawAppIconBuffered(g, APP_PADDING, y + 4, appIndex);
  
  g->setTextColor(selected ? COLOR_ACCENT : COLOR_TEXT);
  g->setTextSize(2);
  g->setCursor(APP_PADDING + APP_ICON_SIZE + 10, y + 8);
  g->print(appNames[appIndex]);
}

// Buffered version of drawAppIcon for double-buffered rendering
void drawAppIconBuffered(Adafruit_GFX* g, int x, int y, int appIndex) {
  uint16_t color = appColors[appIndex];
  if (appIndex == APP_DESKTOP) color = 0x4208;
  
  g->fillRoundRect(x, y, APP_ICON_SIZE, APP_ICON_SIZE, 4, color);
  
  uint16_t iconColor = COLOR_TEXT;
  int cx = x + APP_ICON_SIZE / 2;
  int cy = y + APP_ICON_SIZE / 2;
  
  switch (appIndex) {
    case APP_SETTINGS:
      g->drawCircle(cx, cy, 7, iconColor);
      g->fillCircle(cx, cy, 3, iconColor);
      break;
    case APP_CALC:
      g->drawRect(x + 4, y + 4, 16, 16, iconColor);
      g->drawLine(x + 4, y + 10, x + 19, y + 10, iconColor);
      g->drawLine(x + 10, y + 4, x + 10, y + 19, iconColor);
      break;
    case APP_TIMER:
      g->drawCircle(cx, cy, 9, iconColor);
      g->drawLine(cx, cy, cx, y + 5, iconColor);
      g->drawLine(cx, cy, x + 17, cy, iconColor);
      break;
    case APP_TODO:
      g->drawRect(x + 4, y + 4, 16, 16, iconColor);
      g->drawLine(x + 7, y + 12, x + 10, y + 16, iconColor);
      g->drawLine(x + 10, y + 16, x + 17, y + 7, iconColor);
      break;
    case APP_NOTES:
      g->drawRect(x + 4, y + 3, 16, 18, iconColor);
      g->drawLine(x + 7, y + 7, x + 17, y + 7, iconColor);
      g->drawLine(x + 7, y + 11, x + 17, y + 11, iconColor);
      g->drawLine(x + 7, y + 15, x + 14, y + 15, iconColor);
      break;
    case APP_MUSIC:
      g->fillCircle(x + 8, y + 16, 4, iconColor);
      g->fillRect(x + 11, y + 5, 2, 12, iconColor);
      g->fillRect(x + 11, y + 5, 6, 2, iconColor);
      break;
    case APP_DESKTOP:
      g->fillRect(x + 4, y + 3, 16, 11, 0x6B6D);
      g->drawRect(x + 4, y + 3, 16, 11, iconColor);
      g->fillRect(x + 10, y + 14, 4, 3, 0x6B6D);
      g->fillRect(x + 6, y + 17, 12, 2, 0x6B6D);
      g->drawRect(x + 6, y + 17, 12, 2, iconColor);
      return;
    case APP_FILES:
      g->fillRect(x + 4, y + 5, 8, 3, 0xFDA0);
      g->fillRect(x + 3, y + 7, 18, 12, 0xFFE0);
      g->drawRect(x + 3, y + 7, 18, 12, iconColor);
      g->drawLine(x + 5, y + 11, x + 18, y + 11, 0xFDA0);
      g->drawLine(x + 5, y + 14, x + 18, y + 14, 0xFDA0);
      break;
    case APP_PAINT:
      g->fillRect(x + 14, y + 4, 4, 10, 0x8410);
      g->fillRect(x + 6, y + 12, 10, 6, 0xFDA0);
      g->drawRect(x + 6, y + 12, 10, 6, iconColor);
      g->fillCircle(x + 8, y + 19, 2, 0xF800);
      break;
    case APP_SNAKE:
      g->fillRect(x + 4, y + 6, 6, 4, 0x07E0);
      g->fillRect(x + 8, y + 9, 4, 6, 0x07E0);
      g->fillRect(x + 10, y + 13, 6, 4, 0x07E0);
      g->fillRect(x + 14, y + 13, 5, 4, 0x07E0);
      g->fillRect(x + 17, y + 14, 2, 2, 0x0000);
      g->fillRect(x + 5, y + 14, 4, 4, 0xF800);
      break;
    case APP_TETRIS:
      g->fillRect(x + 4, y + 4, 5, 5, 0xFDA0);
      g->fillRect(x + 4, y + 9, 5, 5, 0xFDA0);
      g->fillRect(x + 9, y + 9, 5, 5, 0xFDA0);
      g->fillRect(x + 14, y + 4, 5, 5, 0xFFE0);
      g->fillRect(x + 14, y + 9, 5, 5, 0xFFE0);
      g->fillRect(x + 4, y + 14, 5, 5, 0x07FF);
      g->fillRect(x + 9, y + 14, 5, 5, 0x07FF);
      g->fillRect(x + 14, y + 14, 5, 5, 0x07FF);
      break;
  }
}

void drawAppDrawer(bool fullRedraw) {
  // Use double buffer for consistent rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  if (fullRedraw) {
    // Draw menu bar to buffer
    g->fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, COLOR_MENUBAR);
    g->setTextColor(COLOR_TEXT);
    g->setTextSize(2);
    g->setCursor(4, 1);
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", hours, minutes);
    g->print(timeStr);
    
    // Clear app area (below menu bar)
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    
    // Draw all visible apps
    for (int i = 0; i < maxVisibleApps && (scrollOffset + i) < NUM_APPS; i++) {
      int appIndex = scrollOffset + i;
      drawAppRowBuffered(g, appIndex, appIndex == selectedApp);
    }
    prevSelectedApp = selectedApp;
  } else {
    // For partial updates, still use full buffered redraw for consistency
    g->fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, COLOR_MENUBAR);
    g->setTextColor(COLOR_TEXT);
    g->setTextSize(2);
    g->setCursor(4, 1);
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", hours, minutes);
    g->print(timeStr);
    
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    for (int i = 0; i < maxVisibleApps && (scrollOffset + i) < NUM_APPS; i++) {
      int appIndex = scrollOffset + i;
      drawAppRowBuffered(g, appIndex, appIndex == selectedApp);
    }
    prevSelectedApp = selectedApp;
  }
  
  // Draw scroll indicator if needed
  if (NUM_APPS > maxVisibleApps) {
    int indicatorHeight = (SCREEN_HEIGHT - MENUBAR_HEIGHT) * maxVisibleApps / NUM_APPS;
    int indicatorY = MENUBAR_HEIGHT + (SCREEN_HEIGHT - MENUBAR_HEIGHT - indicatorHeight) * scrollOffset / (NUM_APPS - maxVisibleApps);
    g->fillRoundRect(SCREEN_WIDTH - 3, indicatorY, 3, indicatorHeight, 1, COLOR_ACCENT);
  }
  
  // Flush buffer to screen
  flushBuffer();
  
  // Redraw notification on top if active (directly to tft)
  if (notifActive) drawNotification();
}

// ===== APP IMPLEMENTATIONS =====

void runApp() {
  // lastButton is already set by the loop before calling this
  switch (selectedApp) {
    case APP_SETTINGS: drawSettingsApp(); break;
    case APP_CALC: drawCalcApp(); break;
    case APP_TIMER: drawTimerApp(); break;
    case APP_TODO: drawTodoApp(); break;
    case APP_NOTES: drawNotesApp(); break;
    case APP_MUSIC: drawMusicApp(); break;
    case APP_DESKTOP: drawDesktopApp(); break;
    case APP_FILES: drawFilesApp(); break;
    case APP_PAINT: drawPaintApp(); break;
    case APP_SNAKE: drawSnakeApp(); break;
    case APP_TETRIS: drawTetrisApp(); break;
    case APP_RESTART: drawRestartApp(); break;
  }
  // Redraw notification on top if active (but not in desktop mode)
  if (notifActive && !desktopMode) drawNotification();
}

// ===== SETTINGS APP =====
void drawSettingsApp() {
  // If in WiFi edit mode, use that screen
  if (wifiEditMode) {
    drawWiFiEditor();
    return;
  }
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  if (lastButton == 1 && settingsSel > 0) settingsSel--;
  if (lastButton == 2 && settingsSel < SETTINGS_ITEMS - 1) settingsSel++;
  
  // Toggle sound with left/right or mid
  if (settingsSel == 0 && (lastButton == 3 || lastButton == 4 || lastButton == 5)) {
    soundEnabled = !soundEnabled;
  }
  // WiFi setup - scan and enter edit mode
  if (settingsSel == 1 && lastButton == 5) {
    wifiEditMode = true;
    scanWiFiNetworks();  // Scan for networks first
    return;
  }
  // Manual time adjust (only if not on WiFi time)
  if (!wifiConnected) {
    if (settingsSel == 2 && lastButton == 3 && hours > 0) hours--;
    if (settingsSel == 2 && lastButton == 4 && hours < 23) hours++;
    if (settingsSel == 3 && lastButton == 3 && minutes > 0) minutes--;
    if (settingsSel == 3 && lastButton == 4 && minutes < 59) minutes++;
  }
  // Boot to Desktop toggle
  if (settingsSel == 4 && (lastButton == 3 || lastButton == 4 || lastButton == 5)) {
    bootToDesktop = !bootToDesktop;
    dataDirty = true;  // Use deferred save instead of immediate save
  }
  // Wipe all saved data
  if (settingsSel == 5 && lastButton == 5) {
    // Clear preferences
    prefs.begin("wifi", false);
    prefs.clear();
    prefs.end();
    // Reset WiFi state
    WiFi.disconnect();
    wifiConnected = false;
    wifiSSID[0] = '\0';
    wifiPass[0] = '\0';
    // Wipe .osdata folder contents
    if (sdCardMounted) {
      SD.remove("/.osdata/notes.txt");
      SD.remove("/.osdata/todo.dat");
      SD.remove("/.osdata/settings.dat");
      SD.remove("/.osdata/wifi.dat");
      // Clear in-memory data too
      notesText[0] = '\0';
      notesCursor = 0;
      for (int i = 0; i < MAX_TODO; i++) todoDone[i] = false;
    }
    // Show confirmation
    showNotification("Data Wiped!");
  }
  
  // Update menu bar time after manual adjustment
  if ((settingsSel == 2 || settingsSel == 3) && (lastButton == 3 || lastButton == 4)) {
    updateMenuBarTime();
  }
  
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(60, MENUBAR_HEIGHT + 2);
  g->print("SETTINGS");
  
  // Focus-based scrolling - calculate visible range based on selection
  int itemH = 20;
  int startY = MENUBAR_HEIGHT + 20;
  int availH = SCREEN_HEIGHT - startY;
  int maxVisible = availH / itemH;
  
  // Auto-scroll to keep selection visible
  if (settingsSel < settingsScrollOffset) {
    settingsScrollOffset = settingsSel;
  } else if (settingsSel >= settingsScrollOffset + maxVisible) {
    settingsScrollOffset = settingsSel - maxVisible + 1;
  }
  
  int y = startY;
  const char* labels[] = {"Sound", "WiFi", "Hour", "Min", "Boot", "Wipe", "v0.1.0"};
  
  for (int i = 0; i < maxVisible && (i + settingsScrollOffset) < SETTINGS_ITEMS; i++) {
    int idx = i + settingsScrollOffset;
    bool sel = (idx == settingsSel);
    
    if (idx == 0) {
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(10, y);
      g->print("Sound: ");
      g->print(soundEnabled ? "ON" : "OFF");
    } else if (idx == 1) {
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(10, y);
      g->print("WiFi: ");
      g->print(wifiConnected ? "OK" : "---");
    } else if (idx == 2) {
      g->setTextColor(sel ? COLOR_ACCENT : (wifiConnected ? 0x6B6D : COLOR_TEXT));
      g->setCursor(10, y);
      g->print("Hour: ");
      g->print(hours);
      if (sel && !wifiConnected) g->print(" <>");
    } else if (idx == 3) {
      g->setTextColor(sel ? COLOR_ACCENT : (wifiConnected ? 0x6B6D : COLOR_TEXT));
      g->setCursor(10, y);
      g->print("Min:  ");
      g->print(minutes);
      if (sel && !wifiConnected) g->print(" <>");
    } else if (idx == 4) {
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(10, y);
      g->print("Boot: ");
      g->print(bootToDesktop ? "ON" : "OFF");
    } else if (idx == 5) {
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(10, y);
      g->print("Wipe Data");
    } else if (idx == 6) {
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(10, y);
      g->print("v1.0 Beta");
    }
    y += itemH;
  }
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== CALCULATOR APP =====
void drawCalcApp() {
  // Handle navigation
  if (lastButton == 3 && calcCol > 0) calcCol--;
  if (lastButton == 4 && calcCol < 3) calcCol++;
  if (lastButton == 1 && calcRow > 0) calcRow--;
  if (lastButton == 2 && calcRow < 3) calcRow++;
  
  // Handle selection
  if (lastButton == 5) {
    int key = calcRow * 4 + calcCol;
    int digits[] = {7, 8, 9, -1, 4, 5, 6, -1, 1, 2, 3, -1, 0, -2, -3, -1};
    int ops[] = {-1, -1, -1, 1, -1, -1, -1, 2, -1, -1, -1, 3, -1, -1, -1, 4};
    
    if (digits[key] >= 0) {
      if (calcNew) { calcValue = digits[key]; calcNew = false; }
      else if (calcValue < 100000) calcValue = calcValue * 10 + digits[key];
    } else if (digits[key] == -2) {
      calcValue = 0; calcStored = 0; calcOp = 0; calcNew = true;
    } else if (digits[key] == -3 && calcOp > 0) {
      switch (calcOp) {
        case 1: calcValue = calcStored + calcValue; break;
        case 2: calcValue = calcStored - calcValue; break;
        case 3: calcValue = calcStored * calcValue; break;
        case 4: calcValue = calcValue ? calcStored / calcValue : 0; break;
      }
      calcOp = 0; calcNew = true;
    } else if (ops[key] > 0) {
      calcStored = calcValue; calcOp = ops[key]; calcNew = true;
    }
  }
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  
  // Display - smaller to fit
  g->drawRect(4, MENUBAR_HEIGHT + 1, 232, 24, COLOR_ACCENT);
  g->setTextSize(2);
  g->setTextColor(COLOR_TEXT);
  char numStr[10];
  sprintf(numStr, "%ld", calcValue);
  g->setCursor(228 - strlen(numStr) * 12, MENUBAR_HEIGHT + 5);
  g->print(numStr);
  
  if (calcOp > 0) {
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(8, MENUBAR_HEIGHT + 5);
    char ops[] = " +-*/";
    g->print(ops[calcOp]);
  }
  
  // Keypad - 4 rows must fit in ~90px (135-18-27=90)
  g->setTextSize(2);
  int startY = MENUBAR_HEIGHT + 27;
  const char* keys[] = {"7","8","9","+", "4","5","6","-", "1","2","3","x", "0","C","=","/"};
  
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      int kx = 4 + c * 59, ky = startY + r * 22;
      bool sel = (r == calcRow && c == calcCol);
      if (sel) g->fillRect(kx, ky, 55, 20, COLOR_HIGHLIGHT);
      g->setTextColor(sel ? COLOR_ACCENT : (c == 3 ? COLOR_CALC : COLOR_TEXT));
      g->setCursor(kx + 20, ky + 2);
      g->print(keys[r * 4 + c]);
    }
  }
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== TIMER APP =====
void drawTimerApp() {
  static int prevMin = -1, prevSec = -1, prevSel = -1;
  static bool prevOn = false, prevPaused = false, needFullDraw = true;
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  // Force full redraw when app is freshly opened
  if (lastButton == 0) needFullDraw = true;
  
  unsigned long now = millis();
  long remaining = 0;
  
  // Timer completion is handled by updateBackgroundTimer()
  
  // Calculate remaining time
  if (timerOn) {
    if (timerPaused) {
      remaining = timerRemaining / 1000;
    } else {
      remaining = (timerEndTime - now) / 1000;
    }
    if (remaining < 0) remaining = 0;
  }
  
  // Handle input
  if (lastButton == 3 && timerSel > 0) timerSel--;
  if (lastButton == 4 && timerSel < 3) timerSel++;
  
  // Adjust time only when stopped
  if (!timerOn && !timerPaused) {
    if (lastButton == 1) {
      if (timerSel == 0 && timerSetMin < 99) timerSetMin++;
      if (timerSel == 1 && timerSetSec < 59) timerSetSec++;
    }
    if (lastButton == 2) {
      if (timerSel == 0 && timerSetMin > 0) timerSetMin--;
      if (timerSel == 1 && timerSetSec > 0) timerSetSec--;
    }
  }
  
  if (lastButton == 5) {
    if (timerSel == 2) {  // Start / Pause / Resume
      if (!timerOn && !timerPaused) {
        // Start new timer
        unsigned long duration = (unsigned long)(timerSetMin * 60 + timerSetSec) * 1000UL;
        timerEndTime = now + duration;
        timerOn = true;
        timerPaused = false;
      } else if (timerOn && !timerPaused) {
        // Pause - save remaining time
        timerRemaining = timerEndTime - now;
        timerPaused = true;
      } else if (timerPaused) {
        // Resume - recalculate end time
        timerEndTime = now + timerRemaining;
        timerPaused = false;
      }
      needFullDraw = true;
    } else if (timerSel == 3) {  // Reset / Cancel
      timerOn = false;
      timerPaused = false;
      timerSetMin = 1;
      timerSetSec = 0;
      needFullDraw = true;
    }
  }
  
  int dispMin, dispSec;
  if (timerOn || timerPaused) {
    dispMin = remaining / 60;
    dispSec = remaining % 60;
  } else {
    dispMin = timerSetMin;
    dispSec = timerSetSec;
  }
  
  // Always do full draw to buffer for screenshot support
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(80, MENUBAR_HEIGHT + 5);
  g->print("TIMER");
  g->setTextColor(COLOR_TEXT);
  g->setTextSize(4);
  g->setCursor(112, MENUBAR_HEIGHT + 30);
  g->print(":");
  
  int y = MENUBAR_HEIGHT + 30;
  
  // Minutes
  g->setTextSize(4);
  g->setTextColor(timerSel == 0 ? COLOR_ACCENT : COLOR_TEXT);
  g->setCursor(40, y);
  if (dispMin < 10) g->print("0");
  g->print(dispMin);
  
  // Seconds
  g->setTextColor(timerSel == 1 ? COLOR_ACCENT : COLOR_TEXT);
  g->setCursor(136, y);
  if (dispSec < 10) g->print("0");
  g->print(dispSec);
  
  // Buttons
  int btnY = SCREEN_HEIGHT - 22;
  g->setTextSize(2);
  g->setTextColor(timerSel == 2 ? COLOR_ACCENT : COLOR_TEXT);
  g->setCursor(15, btnY);
  
  // Button 1: START / PAUSE / RESUME
  if (!timerOn && !timerPaused) {
    g->print("START");
  } else if (timerOn && !timerPaused) {
    g->print("PAUSE");
  } else {
    g->print("RESUME");
  }
  
  // Button 2: RESET / CANCEL
  g->setTextColor(timerSel == 3 ? COLOR_ACCENT : COLOR_TEXT);
  g->setCursor(135, btnY);
  g->print(timerOn || timerPaused ? "CANCEL" : "RESET");
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== TODO APP =====
// Character set for TODO editing (same as WiFi)
const char todoChars[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,!?-'";
#define TODO_NUM_CHARS 69

void drawTodoApp() {
  static int prevSel = -1;
  static bool prevDone[MAX_TODO] = {false, false, false, false};
  static bool needFullDraw = true;
  static bool prevEditMode = false;
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  // Force full redraw when app is freshly opened or mode changes
  if (lastButton == 0) needFullDraw = true;
  if (prevEditMode != todoEditMode) needFullDraw = true;
  prevEditMode = todoEditMode;
  
  if (todoEditMode) {
    // === EDIT MODE - typing like WiFi password ===
    char* currentField = todoItems[todoSel];
    int len = strlen(currentField);
    
    // UP/DOWN - cycle character
    if (lastButton == 1) {
      todoCharIdx = (todoCharIdx + 1) % TODO_NUM_CHARS;
      if (todoCursor < TODO_ITEM_LEN) {
        currentField[todoCursor] = todoChars[todoCharIdx];
        if (todoCursor >= len) currentField[todoCursor + 1] = '\0';
      }
    }
    if (lastButton == 2) {
      todoCharIdx = (todoCharIdx - 1 + TODO_NUM_CHARS) % TODO_NUM_CHARS;
      if (todoCursor < TODO_ITEM_LEN) {
        currentField[todoCursor] = todoChars[todoCharIdx];
        if (todoCursor >= len) currentField[todoCursor + 1] = '\0';
      }
    }
    
    // LEFT/RIGHT - move cursor
    if (lastButton == 3 && todoCursor > 0) todoCursor--;
    if (lastButton == 4 && todoCursor < len) todoCursor++;
    
    // MID - delete character at cursor
    if (lastButton == 5 && todoCursor > 0) {
      int slen = strlen(currentField);
      for (int i = todoCursor; i < slen; i++) {
        currentField[i] = currentField[i + 1];
      }
      todoCursor--;
    }
    
    // Draw edit screen
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    
    int y = MENUBAR_HEIGHT + 4;
    g->setTextSize(2);
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(50, y);
    g->print("Edit TODO #");
    g->print(todoSel + 1);
    y += 24;
    
    // Text field with cursor
    int fieldLen = strlen(currentField);
    int startChar = (todoCursor > 10) ? todoCursor - 10 : 0;
    for (int i = 0; i < 12 && (startChar + i) <= fieldLen; i++) {
      int idx = startChar + i;
      int cx = 10 + i * 18;
      bool isCursor = (idx == todoCursor);
      if (isCursor) g->fillRect(cx, y, 16, 20, COLOR_ACCENT);
      g->setTextColor(isCursor ? COLOR_BG : COLOR_TEXT);
      g->setCursor(cx + 2, y + 2);
      if (idx < fieldLen) g->print(currentField[idx]);
    }
    y += 28;
    
    // Current character indicator
    g->setTextSize(1);
    g->setTextColor(0x6B6D);
    g->setCursor(10, y);
    g->print("Char: ");
    g->setTextColor(COLOR_TEXT);
    g->print(todoChars[todoCharIdx]);
    y += 16;
    
    // Instructions
    g->setTextColor(0x6B6D);
    g->setCursor(5, SCREEN_HEIGHT - 24);
    g->print("UP/DN:Char  L/R:Move  MID:Del");
    g->setCursor(5, SCREEN_HEIGHT - 12);
    g->print("SET:Back to list  RST:Save&Exit");
    
  } else {
    // === LIST MODE ===
    // Handle input - navigate through items + "Add New" option
    int maxSel = todoCount;  // Can select up to todoCount (last is "Add New" if room)
    if (todoCount < MAX_TODO) maxSel = todoCount;  // "Add New" is at index todoCount
    
    if (lastButton == 1 && todoSel > 0) { todoSel--; needFullDraw = true; }
    if (lastButton == 2 && todoSel < maxSel) { todoSel++; needFullDraw = true; }
    
    // MID - toggle done or add new item
    if (lastButton == 5) {
      if (todoSel == todoCount && todoCount < MAX_TODO) {
        // Add new item
        strcpy(todoItems[todoCount], "New task");
        todoDone[todoCount] = false;
        todoCount++;
        dataDirty = true;
        needFullDraw = true;
        playSelect();
      } else if (todoSel < todoCount) {
        // Toggle done
        todoDone[todoSel] = !todoDone[todoSel];
        dataDirty = true;
      }
    }
    
    // RIGHT - enter edit mode (only for existing items)
    if (lastButton == 4 && todoSel < todoCount) {
      todoEditMode = true;
      todoCursor = strlen(todoItems[todoSel]);
      todoCharIdx = 0;
      return;  // Return so edit mode draws on next call
    }
    
    // LEFT - delete item (with confirmation by holding)
    if (lastButton == 3 && todoSel < todoCount && todoCount > 1) {
      // Delete the selected item
      for (int i = todoSel; i < todoCount - 1; i++) {
        strcpy(todoItems[i], todoItems[i + 1]);
        todoDone[i] = todoDone[i + 1];
      }
      todoItems[todoCount - 1][0] = '\0';
      todoDone[todoCount - 1] = false;
      todoCount--;
      if (todoSel >= todoCount) todoSel = todoCount - 1;
      dataDirty = true;
      needFullDraw = true;
      playBack();
    }
    
    // Always do full draw to buffer
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    g->setTextSize(2);
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(5, MENUBAR_HEIGHT + 2);
    g->print("TODO");
    
    // Update count display
    int done = 0;
    for (int i = 0; i < todoCount; i++) if (todoDone[i]) done++;
    g->setTextColor(COLOR_TEXT);
    g->setCursor(180, MENUBAR_HEIGHT + 2);
    g->print(done);
    g->print("/");
    g->print(todoCount);
    
    // Draw items (scrollable, max 4 visible)
    int maxVisible = 4;
    // Adjust scroll to keep selection visible
    if (todoSel < todoScrollOffset) todoScrollOffset = todoSel;
    if (todoSel >= todoScrollOffset + maxVisible) todoScrollOffset = todoSel - maxVisible + 1;
    
    int y = MENUBAR_HEIGHT + 24;
    int displayCount = todoCount + (todoCount < MAX_TODO ? 1 : 0);  // +1 for "Add New" if room
    
    for (int vi = 0; vi < maxVisible && (todoScrollOffset + vi) < displayCount; vi++) {
      int i = todoScrollOffset + vi;
      bool sel = (i == todoSel);
      
      // Clear row
      g->fillRect(0, y, SCREEN_WIDTH, 24, sel ? COLOR_HIGHLIGHT : COLOR_BG);
      
      if (i < todoCount) {
        // Regular item
        // Checkbox
        g->drawRect(8, y + 4, 16, 16, sel ? COLOR_ACCENT : COLOR_TEXT);
        if (todoDone[i]) {
          g->drawLine(10, y + 12, 14, y + 16, COLOR_MESSAGES);
          g->drawLine(14, y + 16, 22, y + 8, COLOR_MESSAGES);
        }
        
        // Text
        g->setTextSize(2);
        g->setTextColor(todoDone[i] ? 0x6B6D : (sel ? COLOR_ACCENT : COLOR_TEXT));
        g->setCursor(30, y + 4);
        g->print(todoItems[i]);
      } else {
        // "Add New" option
        g->setTextSize(2);
        g->setTextColor(sel ? COLOR_ACCENT : 0x07E0);  // Green
        g->setCursor(8, y + 4);
        g->print("+ Add New");
      }
      
      y += 24;
    }
    
    // Scroll indicator
    if (displayCount > maxVisible) {
      int indicatorH = (SCREEN_HEIGHT - MENUBAR_HEIGHT - 34) * maxVisible / displayCount;
      int indicatorY = MENUBAR_HEIGHT + 24 + (SCREEN_HEIGHT - MENUBAR_HEIGHT - 34 - indicatorH) * todoScrollOffset / (displayCount - maxVisible);
      g->fillRect(SCREEN_WIDTH - 3, MENUBAR_HEIGHT + 24, 3, SCREEN_HEIGHT - MENUBAR_HEIGHT - 34, COLOR_BG);
      g->fillRoundRect(SCREEN_WIDTH - 3, indicatorY, 3, indicatorH, 1, COLOR_ACCENT);
    }
    
    // Hint at bottom
    g->setTextSize(1);
    g->setTextColor(0x6B6D);
    g->setCursor(5, SCREEN_HEIGHT - 10);
    g->print("MID:Toggle/Add  R:Edit  L:Del");
  }
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== NOTES APP (Text Editor) =====
void drawNotesApp() {
  static int prevCursor = -1, prevCharIdx = -1;
  static char prevText[NOTE_LEN + 1] = "";
  static bool needFullDraw = true;
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  // Force full redraw when app is freshly opened
  if (lastButton == 0) needFullDraw = true;
  
  int oldCursor = notesCursor;
  
  // Handle input - LEFT/RIGHT move cursor only (keep character selector unchanged)
  if (lastButton == 3 && notesCursor > 0) {
    notesCursor--;
  }
  if (lastButton == 4 && notesCursor < NOTE_LEN - 1) {
    notesCursor++;
  }
  if (lastButton == 1) {
    noteCharIdx = (noteCharIdx + NUM_CHARS - 1) % NUM_CHARS;
    notesText[notesCursor] = noteChars[noteCharIdx];
  }
  if (lastButton == 2) {
    noteCharIdx = (noteCharIdx + 1) % NUM_CHARS;
    notesText[notesCursor] = noteChars[noteCharIdx];
  }
  if (lastButton == 5) {
    notesText[notesCursor] = ' ';
    noteCharIdx = 26;
  }
  
  int y = MENUBAR_HEIGHT + 30;
  
  // Always do full draw to buffer for screenshot support
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(70, MENUBAR_HEIGHT + 5);
  g->print("NOTES");
  g->drawRect(4, y - 2, 232, 36, COLOR_TEXT);
  g->setTextColor(0x6B6D);
  g->setCursor(10, MENUBAR_HEIGHT + 75);
  g->print("Char:");
  g->setCursor(5, SCREEN_HEIGHT - 20);
  g->print("U/D=Chr L/R=Mv M=Del");
  
  // Draw all characters
  g->setTextSize(2);
  for (int i = 0; i < NOTE_LEN; i++) {
    bool isCursor = (i == notesCursor);
    int cx = 8 + i * 12;
    if (isCursor) g->fillRect(cx, y + 4, 12, 20, COLOR_ACCENT);
    g->setTextColor(isCursor ? COLOR_BG : COLOR_TEXT);
    g->setCursor(cx, y + 8);
    g->print(notesText[i]);
  }
  
  // Character selector
  g->setTextSize(3);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(80, MENUBAR_HEIGHT + 71);
  g->print(noteChars[noteCharIdx]);
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== MUSIC APP =====
void playNote(int freq, int dur) {
  if (freq > 0) ledcWriteTone(BUZZER_PIN, freq);
  else ledcWriteTone(BUZZER_PIN, 0);
  delay(dur + 20);
  ledcWriteTone(BUZZER_PIN, 0);
}

void drawMusicApp() {
  static int musicScroll = 0;
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  if (lastButton == 1 && musicTrack > 0) musicTrack--;
  if (lastButton == 2 && musicTrack < NUM_TRACKS - 1) musicTrack++;
  if (lastButton == 5) {
    switch (musicTrack) {
      case 0:  // Twinkle Twinkle
        playNote(NOTE_C4, 200); playNote(NOTE_C4, 200);
        playNote(NOTE_G4, 200); playNote(NOTE_G4, 200);
        playNote(NOTE_A4, 200); playNote(NOTE_A4, 200);
        playNote(NOTE_G4, 400);
        playNote(NOTE_F4, 200); playNote(NOTE_F4, 200);
        playNote(NOTE_E4, 200); playNote(NOTE_E4, 200);
        playNote(NOTE_D4, 200); playNote(NOTE_D4, 200);
        playNote(NOTE_C4, 400);
        break;
      case 1:  // Happy Birthday
        playNote(NOTE_C4, 150); playNote(NOTE_C4, 150);
        playNote(NOTE_D4, 300); playNote(NOTE_C4, 300);
        playNote(NOTE_F4, 300); playNote(NOTE_E4, 500);
        playNote(NOTE_C4, 150); playNote(NOTE_C4, 150);
        playNote(NOTE_D4, 300); playNote(NOTE_C4, 300);
        playNote(NOTE_G4, 300); playNote(NOTE_F4, 500);
        break;
      case 2:  // Fur Elise (simplified)
        playNote(NOTE_E5, 150); playNote(NOTE_DS4, 150);
        playNote(NOTE_E5, 150); playNote(NOTE_DS4, 150);
        playNote(NOTE_E5, 150); playNote(NOTE_B4, 150);
        playNote(NOTE_D5, 150); playNote(NOTE_C5, 150);
        playNote(NOTE_A4, 300);
        break;
      case 3:  // Scale
        for (int f = 262; f <= 523; f += 40) {
          ledcWriteTone(BUZZER_PIN, f);
          delay(100);
        }
        break;
      case 4:  // Alert
        for (int i = 0; i < 3; i++) {
          ledcWriteTone(BUZZER_PIN, 880);
          delay(100);
          ledcWriteTone(BUZZER_PIN, 0);
          delay(50);
        }
        break;
    }
    ledcWriteTone(BUZZER_PIN, 0);
  }
  
  // Scroll to keep selection visible (3 visible items max)
  int maxVisible = 3;
  if (musicTrack < musicScroll) musicScroll = musicTrack;
  if (musicTrack >= musicScroll + maxVisible) musicScroll = musicTrack - maxVisible + 1;
  
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(70, MENUBAR_HEIGHT + 5);
  g->print("MUSIC");
  
  int y = MENUBAR_HEIGHT + 30;
  for (int i = musicScroll; i < NUM_TRACKS && i < musicScroll + maxVisible; i++) {
    bool sel = (i == musicTrack);
    if (sel) g->fillRect(0, y, SCREEN_WIDTH, 26, COLOR_HIGHLIGHT);
    
    g->setTextSize(2);
    g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
    g->setCursor(sel ? 8 : 20, y + 4);
    if (sel) g->print("> ");
    g->print(trackNames[i]);
    y += 28;
  }
  
  // Show scroll indicators
  if (musicScroll > 0) {
    g->setCursor(SCREEN_WIDTH - 20, MENUBAR_HEIGHT + 30);
    g->setTextColor(0x6B6D);
    g->print("^");
  }
  if (musicScroll + maxVisible < NUM_TRACKS) {
    g->setCursor(SCREEN_WIDTH - 20, y - 10);
    g->setTextColor(0x6B6D);
    g->print("v");
  }
  
  g->setTextColor(0x6B6D);
  g->setCursor(50, SCREEN_HEIGHT - 20);
  g->print("MID = Play");
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== DESKTOP MODE APP =====

// Cursor type: 0=arrow, 1=grab hand
int cursorType = 0;

// Draw mouse pointer (arrow cursor or grab hand)
void drawMousePointer(int16_t x, int16_t y, bool erase) {
  uint16_t col = erase ? DESK_DESKTOP_COLOR : 0xFFFF;
  uint16_t outline = erase ? DESK_DESKTOP_COLOR : 0x0000;
  
  // Check if over taskbar for proper erase color
  if (erase && y >= SCREEN_HEIGHT - DESK_TASKBAR_H) {
    col = DESK_TASKBAR_COLOR;
    outline = DESK_TASKBAR_COLOR;
  }
  
  // Determine cursor type based on dragging/resizing state
  bool showGrab = (dragging || resizing) && !erase;
  
  if (showGrab) {
    // Grab hand cursor (closed fist)
    // Palm
    gfx->fillRect(x, y + 4, 9, 7, col);
    gfx->drawRect(x, y + 4, 9, 7, outline);
    // Fingers (bent)
    gfx->fillRect(x + 1, y + 1, 2, 4, col);
    gfx->drawRect(x + 1, y + 1, 2, 4, outline);
    gfx->fillRect(x + 3, y, 2, 5, col);
    gfx->drawRect(x + 3, y, 2, 5, outline);
    gfx->fillRect(x + 5, y, 2, 5, col);
    gfx->drawRect(x + 5, y, 2, 5, outline);
    gfx->fillRect(x + 7, y + 2, 2, 3, col);
    gfx->drawRect(x + 7, y + 2, 2, 3, outline);
  } else {
    // Arrow pointer shape (11 pixels tall)
    // Outline first, then fill
    if (!erase) {
      // Black outline
      gfx->drawLine(x, y, x, y + 10, outline);
      gfx->drawLine(x, y, x + 7, y + 7, outline);
      gfx->drawLine(x, y + 10, x + 3, y + 7, outline);
      gfx->drawLine(x + 3, y + 7, x + 7, y + 7, outline);
    }
    // White fill
    gfx->drawLine(x + 1, y + 2, x + 1, y + 8, col);
    gfx->drawLine(x + 2, y + 3, x + 2, y + 7, col);
    gfx->drawLine(x + 3, y + 4, x + 3, y + 6, col);
    gfx->drawPixel(x + 4, y + 5, col);
    gfx->drawPixel(x + 4, y + 6, col);
    gfx->drawPixel(x + 5, y + 6, col);
  }
}

// Check if pointer hits a point (with larger hitbox centered on pointer)
// The pointer hotspot is offset 3 pixels down and 2 right from mouseX/mouseY
// and uses a 6x6 hitbox for easier clicking
bool deskPtrHits(int16_t px, int16_t py, int16_t x, int16_t y, int16_t w, int16_t h) {
  // Offset pointer position to center of visual pointer (approx 3 down, 2 right)
  int16_t hx = px + 2;  // Hotspot X
  int16_t hy = py + 3;  // Hotspot Y
  // Use a 6x6 hitbox around the hotspot for easier clicking
  return (hx + 3 >= x && hx - 3 < x + w && hy + 3 >= y && hy - 3 < y + h);
}

// Check if pointer hits start button
bool deskPtrHitsStart() {
  return deskPtrHits(mouseX, mouseY, 2, SCREEN_HEIGHT - DESK_TASKBAR_H + 3, DESK_START_W, DESK_TASKBAR_H - 6);
}

// Check if pointer hits start menu (two columns)
int deskPtrHitsStartMenu() {
  if (!startMenuOpen) return -1;
  int colW = 52;
  int itemH = 14;
  int rows = (START_MENU_ITEMS + 1) / 2;
  int menuW = colW * 2 + 6;
  int menuH = rows * itemH + 6;
  int menuY = SCREEN_HEIGHT - DESK_TASKBAR_H - menuH;
  if (deskPtrHits(mouseX, mouseY, 2, menuY, menuW, menuH)) {
    int col = (mouseX - 4) / colW;  // 0 or 1
    int row = (mouseY - menuY - 3) / itemH;
    if (col >= 0 && col < 2 && row >= 0 && row < rows) {
      int idx = col * rows + row;
      if (idx >= 0 && idx < START_MENU_ITEMS) return idx;
    }
  }
  return -1;
}

// Find which window the pointer is over (returns index in deskWinOrder, -1 if none)
int8_t deskPtrHitsWindow() {
  for (int8_t i = 0; i < deskWinCount; i++) {
    int8_t idx = deskWinOrder[i];
    if (deskWins[idx].open) {
      DeskWindow& w = deskWins[idx];
      int16_t wx, wy, ww, wh;
      if (w.fullscreen) {
        wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
      } else if (w.maximized) {
        wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
      } else {
        wx = w.x; wy = w.y; ww = w.w; wh = w.h;
      }
      if (deskPtrHits(mouseX, mouseY, wx, wy, ww, wh)) {
        return i;
      }
    }
  }
  return -1;
}

// Check if pointer hits window title bar
bool deskPtrHitsTitleBar(int8_t winIdx) {
  if (winIdx < 0) return false;
  DeskWindow& w = deskWins[winIdx];
  if (w.fullscreen) return false;  // No title bar in fullscreen
  int16_t wx = w.maximized ? 0 : w.x;
  int16_t wy = w.maximized ? 0 : w.y;
  int16_t ww = w.maximized ? SCREEN_WIDTH : w.w;
  return deskPtrHits(mouseX, mouseY, wx, wy, ww - 2 * DESK_BTN_SIZE - 4, DESK_TITLEBAR_H);
}

// Check if pointer hits close button
bool deskPtrHitsClose(int8_t winIdx) {
  if (winIdx < 0) return false;
  DeskWindow& w = deskWins[winIdx];
  if (w.fullscreen) return false;  // No close button in fullscreen
  int16_t wx = w.maximized ? 0 : w.x;
  int16_t wy = w.maximized ? 0 : w.y;
  int16_t ww = w.maximized ? SCREEN_WIDTH : w.w;
  return deskPtrHits(mouseX, mouseY, wx + ww - DESK_BTN_SIZE - 2, wy + 1, DESK_BTN_SIZE, DESK_BTN_SIZE);
}

// Check if pointer hits maximize button
bool deskPtrHitsMaximize(int8_t winIdx) {
  if (winIdx < 0) return false;
  DeskWindow& w = deskWins[winIdx];
  if (w.fullscreen) return false;  // No maximize button in fullscreen
  int16_t wx = w.maximized ? 0 : w.x;
  int16_t wy = w.maximized ? 0 : w.y;
  int16_t ww = w.maximized ? SCREEN_WIDTH : w.w;
  return deskPtrHits(mouseX, mouseY, wx + ww - 2 * DESK_BTN_SIZE - 4, wy + 1, DESK_BTN_SIZE, DESK_BTN_SIZE);
}

// Check if pointer hits resize corner (bottom-right, on outside edge of window)
bool deskPtrHitsResize(int8_t winIdx) {
  if (winIdx < 0) return false;
  DeskWindow& w = deskWins[winIdx];
  if (w.maximized || w.fullscreen) return false;  // No resize in maximized or fullscreen
  // Resize grip hitbox - larger area (12x12) for easier clicking
  return deskPtrHits(mouseX, mouseY, w.x + w.w - 4, w.y + w.h - 4, 12, 12);
}

// Check if pointer hits "Back" icon on desktop
bool deskPtrHitsBackIcon() {
  return deskPtrHits(mouseX, mouseY, SCREEN_WIDTH - 28, 4, 24, 24);
}

// Bring window to front
void deskBringToFront(int8_t orderIdx) {
  if (orderIdx <= 0) return;
  int8_t winIdx = deskWinOrder[orderIdx];
  for (int8_t i = orderIdx; i > 0; i--) {
    deskWinOrder[i] = deskWinOrder[i - 1];
  }
  deskWinOrder[0] = winIdx;
  activeWin = winIdx;
}

// Open a new desktop window
void deskOpenWindow(uint8_t appType, const char* title) {
  // Find free slot
  int8_t slot = -1;
  for (int8_t i = 0; i < MAX_DESK_WINDOWS; i++) {
    if (!deskWins[i].open) { slot = i; break; }
  }
  if (slot < 0) return;  // No free slots
  
  DeskWindow& w = deskWins[slot];
  w.open = true;
  w.appType = appType;
  w.maximized = false;
  w.fullscreen = false;
  strncpy(w.title, title, 11);
  w.title[11] = '\0';
  
  // Set size based on app type - sized for mouse interaction
  switch (appType) {
    case 0:  // Settings
      w.w = 85; w.h = 70; w.minW = 70; w.minH = 60;
      break;
    case 1:  // Calc - needs space for keypad
      w.w = 95; w.h = 85; w.minW = 95; w.minH = 85;
      break;
    case 2:  // Timer - needs +/- buttons
      w.w = 90; w.h = 65; w.minW = 80; w.minH = 55;
      break;
    case 3:  // TODO
      w.w = 85; w.h = 60; w.minW = 70; w.minH = 50;
      break;
    case 4:  // Notes - needs virtual keyboard
      w.w = 130; w.h = 100; w.minW = 120; w.minH = 90;
      break;
    case 5:  // Music
      w.w = 85; w.h = 55; w.minW = 70; w.minH = 45;
      break;
    case 6:  // Files
      w.w = 180; w.h = 100; w.minW = 140; w.minH = 80;
      break;
    case 7:  // Editor - virtual keyboard like Notes
      w.w = 130; w.h = 100; w.minW = 120; w.minH = 90;
      break;
    case 8:  // WiFi - virtual keyboard for password entry
      w.w = 130; w.h = 100; w.minW = 120; w.minH = 90;
      break;
    case 9:  // Tetris - game window
      w.w = 100; w.h = 120; w.minW = 100; w.minH = 120;
      break;
    case 10:  // Paint - drawing canvas
      w.w = 120; w.h = 100; w.minW = 100; w.minH = 80;
      break;
    case 11:  // Snake - game window
      w.w = 100; w.h = 120; w.minW = 100; w.minH = 120;
      break;
    case 12:  // Image viewer - opens in true fullscreen mode
      w.w = 160; w.h = 110; w.minW = 120; w.minH = 80;
      w.fullscreen = true;  // Open in true fullscreen (no title bar)
      break;
    default:
      w.w = 80; w.h = 50; w.minW = 50; w.minH = 30;
  }
  
  // Position with slight offset from previous windows
  w.x = 20 + (slot * 15) % 60;
  w.y = 10 + (slot * 12) % 40;
  
  // Add to z-order (front)
  for (int8_t i = deskWinCount; i > 0; i--) {
    deskWinOrder[i] = deskWinOrder[i - 1];
  }
  deskWinOrder[0] = slot;
  deskWinCount++;
  activeWin = slot;
}

// Close a desktop window
void deskCloseWindow(int8_t winIdx) {
  if (winIdx < 0 || !deskWins[winIdx].open) return;
  
  // Reset Paint canvas when closing Paint window (appType 10)
  // Next time Paint opens, it will be a fresh session
  if (deskWins[winIdx].appType == 10) {
    paintInitCanvas();  // Reset canvas and start new session
  }
  
  // Reset BMP viewer when closing Image window (appType 12)
  // Note: bmpViewBuffer is kept allocated to avoid fragmentation
  if (deskWins[winIdx].appType == 12) {
    viewingBMP = false;
    bmpViewWidth = 0;
    bmpViewHeight = 0;
    imgViewerOverlayVisible = false;
  }
  
  deskWins[winIdx].open = false;
  
  // Remove from z-order
  int8_t orderPos = -1;
  for (int8_t i = 0; i < deskWinCount; i++) {
    if (deskWinOrder[i] == winIdx) { orderPos = i; break; }
  }
  if (orderPos >= 0) {
    for (int8_t i = orderPos; i < deskWinCount - 1; i++) {
      deskWinOrder[i] = deskWinOrder[i + 1];
    }
    deskWinCount--;
  }
  
  // Set new active window
  activeWin = (deskWinCount > 0) ? deskWinOrder[0] : -1;
}

// Draw a single window
void deskDrawWindow(int8_t winIdx) {
  if (winIdx < 0 || !deskWins[winIdx].open) return;
  yield();  // Prevent watchdog timeout during window drawing
  DeskWindow& w = deskWins[winIdx];
  bool active = (winIdx == activeWin);
  
  int16_t x = w.x, y = w.y, ww = w.w, wh = w.h;
  if (w.fullscreen) {
    // True fullscreen - no title bar, covers entire screen including taskbar
    x = 0; y = 0;
    ww = SCREEN_WIDTH;
    wh = SCREEN_HEIGHT;
  } else if (w.maximized) {
    x = 0; y = 0;
    ww = SCREEN_WIDTH;
    wh = SCREEN_HEIGHT;  // Fill entire screen including taskbar area
  }
  
  // Skip border and title bar for fullscreen
  if (!w.fullscreen) {
    // Window with 3D raised border (Win98 style)
    // Outer border - black
    gfx->drawRect(x, y, ww, wh, 0x0000);
    // Fill window background
    gfx->fillRect(x + 1, y + 1, ww - 2, wh - 2, DESK_WINDOW_COLOR);
    // 3D raised edge - light on top/left
    gfx->drawFastHLine(x + 1, y + 1, ww - 2, DESK_BTN_LIGHT);
    gfx->drawFastVLine(x + 1, y + 1, wh - 2, DESK_BTN_LIGHT);
    // 3D raised edge - dark on bottom/right (inner)
    gfx->drawFastHLine(x + 2, y + wh - 2, ww - 3, DESK_BTN_DARK);
    gfx->drawFastVLine(x + ww - 2, y + 2, wh - 3, DESK_BTN_DARK);
    
    // Title bar with gradient effect (Win98 blue gradient)
    uint16_t tbColor = active ? DESK_TITLEBAR_COLOR : DESK_TITLEBAR_INACTIVE;
    gfx->fillRect(x + 2, y + 2, ww - 4, DESK_TITLEBAR_H - 2, tbColor);
    
    // Title text
    gfx->setTextSize(1);
    gfx->setTextColor(active ? 0xFFFF : 0xC618);
    gfx->setCursor(x + 4, y + 5);
    gfx->print(w.title);
    
    // Close button with 3D effect
    int bx = x + ww - DESK_BTN_SIZE - 3;
    int by = y + 2;
    gfx->fillRect(bx, by, DESK_BTN_SIZE, DESK_BTN_SIZE - 2, DESK_BTN_FACE);
    gfx->drawFastHLine(bx, by, DESK_BTN_SIZE, DESK_BTN_LIGHT);
    gfx->drawFastVLine(bx, by, DESK_BTN_SIZE - 2, DESK_BTN_LIGHT);
    gfx->drawFastHLine(bx, by + DESK_BTN_SIZE - 3, DESK_BTN_SIZE, DESK_BTN_DARK);
    gfx->drawFastVLine(bx + DESK_BTN_SIZE - 1, by, DESK_BTN_SIZE - 2, DESK_BTN_DARK);
    // X icon
    gfx->drawLine(bx + 3, by + 2, bx + DESK_BTN_SIZE - 4, by + DESK_BTN_SIZE - 5, 0x0000);
    gfx->drawLine(bx + 3, by + DESK_BTN_SIZE - 5, bx + DESK_BTN_SIZE - 4, by + 2, 0x0000);
    
    // Maximize button with 3D effect
    bx = x + ww - 2 * DESK_BTN_SIZE - 5;
    gfx->fillRect(bx, by, DESK_BTN_SIZE, DESK_BTN_SIZE - 2, DESK_BTN_FACE);
    gfx->drawFastHLine(bx, by, DESK_BTN_SIZE, DESK_BTN_LIGHT);
    gfx->drawFastVLine(bx, by, DESK_BTN_SIZE - 2, DESK_BTN_LIGHT);
    gfx->drawFastHLine(bx, by + DESK_BTN_SIZE - 3, DESK_BTN_SIZE, DESK_BTN_DARK);
    gfx->drawFastVLine(bx + DESK_BTN_SIZE - 1, by, DESK_BTN_SIZE - 2, DESK_BTN_DARK);
    // Square icon
    gfx->drawRect(bx + 3, by + 2, DESK_BTN_SIZE - 6, DESK_BTN_SIZE - 6, 0x0000);
    gfx->drawFastHLine(bx + 3, by + 3, DESK_BTN_SIZE - 6, 0x0000);
  }
  
  // Window content area with sunken 3D border
  int cx, cy, cw, ch;
  if (w.fullscreen) {
    cx = 0; cy = 0; cw = SCREEN_WIDTH; ch = SCREEN_HEIGHT;
  } else {
    cx = x + 3; cy = y + DESK_TITLEBAR_H;
    cw = ww - 6; ch = wh - DESK_TITLEBAR_H - 3;
  }
  // Sunken content area - dark background with Win98 style 3D border
  // For fullscreen image viewer, the image will be drawn over this
  gfx->fillRect(cx, cy, cw, ch, 0x0000);  // Black content background
  if (!w.fullscreen) {
    gfx->drawFastHLine(cx, cy, cw, DESK_BTN_DARK);
    gfx->drawFastVLine(cx, cy, ch, DESK_BTN_DARK);
    gfx->drawFastHLine(cx, cy + ch - 1, cw, DESK_BTN_LIGHT);
    gfx->drawFastVLine(cx + cw - 1, cy, ch, DESK_BTN_LIGHT);
  }
  
  // Draw actual app content in window - mouse interactive
  // Keep text size 1 for both modes - just scale UI elements slightly when maximized
  int textSize = 1;
  int charW = 6;
  int charH = 8;
  gfx->setTextSize(textSize);
  
  switch (w.appType) {
    case 0:  // Settings - clickable menu items with scrollbar (Win98 style)
      {
        int itemH = w.maximized ? 14 : 10;
        int scrollW = 8;
        int maxVisible = (ch - 2) / itemH;
        const char* labels[] = {"Sound", "WiFi", "Hour", "Min", "BTD", "Wipe", "v1.0"};
        for (int i = 0; i < maxVisible && (i + settingsScrollOffset) < SETTINGS_ITEMS; i++) {
          int idx = i + settingsScrollOffset;
          int iy = cy + 2 + i * itemH;
          if (iy + itemH > cy + ch) break;
          // Draw clickable row - white text on black background
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(cx + 2, iy);
          gfx->print(labels[idx]);
          gfx->print(":");
          // Draw value/button on right - Win98 3D buttons
          int vx = cx + (w.maximized ? 52 : 36);
          int btnW = w.maximized ? 28 : 20;
          int btnH = w.maximized ? 10 : 8;
          int smallBtnW = w.maximized ? 14 : 10;
          if (idx == 0) {
            // 3D button style
            gfx->fillRect(vx, iy, btnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx, iy, btnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx, iy + btnH - 1, btnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + btnW - 1, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(soundEnabled ? 0x0400 : 0xF800);
            gfx->setCursor(vx + 2, iy + 1);
            gfx->print(soundEnabled ? "ON" : "OFF");
          } else if (idx == 1) {
            gfx->setTextColor(wifiConnected ? 0x0400 : 0x8410);
            gfx->print(wifiConnected ? "OK" : "--");
          } else if (idx == 2) {
            // 3D minus button
            gfx->fillRect(vx, iy, smallBtnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx, iy, smallBtnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx, iy + btnH - 1, smallBtnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + smallBtnW - 1, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(vx + 3, iy + 1); gfx->print("-");
            gfx->setTextColor(0xFFFF);  // White for value on black bg
            gfx->setCursor(vx + smallBtnW + 4, iy + 1); gfx->print(hours);
            // 3D plus button
            gfx->fillRect(vx + smallBtnW * 2 + 8, iy, smallBtnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx + smallBtnW * 2 + 8, iy, smallBtnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx + smallBtnW * 2 + 8, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx + smallBtnW * 2 + 8, iy + btnH - 1, smallBtnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + smallBtnW * 3 + 7, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(vx + smallBtnW * 2 + 11, iy + 1); gfx->print("+");
          } else if (idx == 3) {
            // 3D minus button
            gfx->fillRect(vx, iy, smallBtnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx, iy, smallBtnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx, iy + btnH - 1, smallBtnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + smallBtnW - 1, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(vx + 3, iy + 1); gfx->print("-");
            gfx->setTextColor(0xFFFF);  // White for value on black bg
            gfx->setCursor(vx + smallBtnW + 4, iy + 1); gfx->print(minutes);
            // 3D plus button
            gfx->fillRect(vx + smallBtnW * 2 + 8, iy, smallBtnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx + smallBtnW * 2 + 8, iy, smallBtnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx + smallBtnW * 2 + 8, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx + smallBtnW * 2 + 8, iy + btnH - 1, smallBtnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + smallBtnW * 3 + 7, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(vx + smallBtnW * 2 + 11, iy + 1); gfx->print("+");
          } else if (idx == 4) {  // Boot to Desktop toggle - 3D button
            gfx->fillRect(vx, iy, btnW, btnH, DESK_BTN_FACE);
            gfx->drawFastHLine(vx, iy, btnW, DESK_BTN_LIGHT);
            gfx->drawFastVLine(vx, iy, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(vx, iy + btnH - 1, btnW, DESK_BTN_DARK);
            gfx->drawFastVLine(vx + btnW - 1, iy, btnH, DESK_BTN_DARK);
            gfx->setTextColor(bootToDesktop ? 0x0400 : 0xF800);
            gfx->setCursor(vx + 2, iy + 1);
            gfx->print(bootToDesktop ? "ON" : "OFF");
          } else if (idx == 5) {  // Wipe - 3D red button
            gfx->fillRect(vx, iy, btnW + 20, btnH, 0xFBE0);  // Light red/pink
            gfx->drawFastHLine(vx, iy, btnW + 20, 0xFFFF);
            gfx->drawFastVLine(vx, iy, btnH, 0xFFFF);
            gfx->drawFastHLine(vx, iy + btnH - 1, btnW + 20, 0xA000);
            gfx->drawFastVLine(vx + btnW + 19, iy, btnH, 0xA000);
            gfx->setTextColor(0x8000);
            gfx->setCursor(vx + 4, iy + 1);
            gfx->print("WIPE");
          }
        }
        // Vertical scrollbar - Win98 style
        if (SETTINGS_ITEMS > maxVisible) {
          int sbX = cx + cw - scrollW - 1;
          int sbY = cy + 2;
          int sbH = ch - 4;
          gfx->fillRect(sbX, sbY, scrollW, sbH, DESK_BTN_FACE);
          gfx->drawFastHLine(sbX, sbY, scrollW, DESK_BTN_DARK);
          gfx->drawFastVLine(sbX, sbY, sbH, DESK_BTN_DARK);
          int thumbH = max(10, sbH * maxVisible / SETTINGS_ITEMS);
          int thumbY = sbY + (sbH - thumbH) * settingsScrollOffset / max(1, SETTINGS_ITEMS - maxVisible);
          // 3D thumb
          gfx->fillRect(sbX + 1, thumbY, scrollW - 2, thumbH, DESK_BTN_FACE);
          gfx->drawFastHLine(sbX + 1, thumbY, scrollW - 2, DESK_BTN_LIGHT);
          gfx->drawFastVLine(sbX + 1, thumbY, thumbH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(sbX + 1, thumbY + thumbH - 1, scrollW - 2, DESK_BTN_DARK);
          gfx->drawFastVLine(sbX + scrollW - 2, thumbY, thumbH, DESK_BTN_DARK);
        }
      }
      break;
    case 1:  // Calc - Win98 style keypad
      {
        // Display area - sunken 3D style
        int dispH = w.maximized ? 14 : 11;
        gfx->fillRect(cx + 1, cy + 1, cw - 2, dispH, 0xFFFF);
        gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1, cy + 1, dispH, DESK_BTN_DARK);
        gfx->drawFastHLine(cx + 1, cy + dispH, cw - 2, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + cw - 2, cy + 1, dispH, DESK_BTN_LIGHT);
        gfx->setTextColor(0x0000);
        char numStr[12];
        sprintf(numStr, "%ld", calcValue);
        int tw = strlen(numStr) * charW;
        gfx->setCursor(cx + cw - 4 - tw, cy + 3);
        gfx->print(numStr);
        if (calcOp > 0) {
          gfx->setTextColor(0x000F);  // Dark blue for operator
          gfx->setCursor(cx + 3, cy + 3);
          char ops[] = " +-*/";
          gfx->print(ops[calcOp]);
        }
        // Keypad - 4x4 grid with 3D buttons
        int kw = w.maximized ? 28 : 21;
        int kh = w.maximized ? 16 : 14;
        int startY = cy + dispH + 2;
        const char* keys[] = {"7","8","9","+", "4","5","6","-", "1","2","3","*", "0","C","=","/"};
        for (int r = 0; r < 4; r++) {
          for (int c = 0; c < 4; c++) {
            int kx = cx + 2 + c * kw;
            int ky = startY + r * kh;
            bool isOp = (c == 3);
            // 3D button
            gfx->fillRect(kx, ky, kw - 2, kh - 2, isOp ? 0xFD20 : DESK_BTN_FACE);
            gfx->drawFastHLine(kx, ky, kw - 2, DESK_BTN_LIGHT);
            gfx->drawFastVLine(kx, ky, kh - 2, DESK_BTN_LIGHT);
            gfx->drawFastHLine(kx, ky + kh - 3, kw - 2, DESK_BTN_DARK);
            gfx->drawFastVLine(kx + kw - 3, ky, kh - 2, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(kx + kw/2 - charW/2 - 1, ky + (kh - charH)/2 - 1);
            gfx->print(keys[r * 4 + c]);
          }
        }
      }
      break;
    case 2:  // Timer - Win98 style buttons
      {
        // Calculate display time
        int dispMin = timerSetMin, dispSec = timerSetSec;
        if (timerOn || timerPaused) {
          long remaining = 0;
          if (timerPaused) remaining = timerRemaining / 1000;
          else remaining = (timerEndTime - millis()) / 1000;
          if (remaining < 0) remaining = 0;
          dispMin = remaining / 60;
          dispSec = remaining % 60;
        }
        // Time display - sunken 3D style
        int timeDispH = w.maximized ? 18 : 14;
        gfx->fillRect(cx + 1, cy + 1, cw - 2, timeDispH, 0xFFFF);
        gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1, cy + 1, timeDispH, DESK_BTN_DARK);
        gfx->drawFastHLine(cx + 1, cy + timeDispH, cw - 2, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + cw - 2, cy + 1, timeDispH, DESK_BTN_LIGHT);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + 20, cy + 4);
        if (dispMin < 10) gfx->print("0");
        gfx->print(dispMin);
        gfx->print(":");
        if (dispSec < 10) gfx->print("0");
        gfx->print(dispSec);
        // +/- buttons for minutes - 3D style
        int tBtnW = w.maximized ? 18 : 14;
        int tBtnH = w.maximized ? 12 : 10;
        int tBtnY = cy + timeDispH + 4;
        // M- button
        gfx->fillRect(cx + 2, tBtnY, tBtnW, tBtnH, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + 2, tBtnY, tBtnW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 2, tBtnY, tBtnH, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 2, tBtnY + tBtnH - 1, tBtnW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1 + tBtnW, tBtnY, tBtnH, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + 4, tBtnY + 2); gfx->print("M-");
        // M+ button
        gfx->fillRect(cx + 4 + tBtnW, tBtnY, tBtnW, tBtnH, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + 4 + tBtnW, tBtnY, tBtnW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 4 + tBtnW, tBtnY, tBtnH, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 4 + tBtnW, tBtnY + tBtnH - 1, tBtnW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 3 + tBtnW * 2, tBtnY, tBtnH, DESK_BTN_DARK);
        gfx->setCursor(cx + 6 + tBtnW, tBtnY + 2); gfx->print("M+");
        // S- button
        gfx->fillRect(cx + 6 + tBtnW * 2, tBtnY, tBtnW, tBtnH, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + 6 + tBtnW * 2, tBtnY, tBtnW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 6 + tBtnW * 2, tBtnY, tBtnH, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 6 + tBtnW * 2, tBtnY + tBtnH - 1, tBtnW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 5 + tBtnW * 3, tBtnY, tBtnH, DESK_BTN_DARK);
        gfx->setCursor(cx + 8 + tBtnW * 2, tBtnY + 2); gfx->print("S-");
        // S+ button
        gfx->fillRect(cx + 8 + tBtnW * 3, tBtnY, tBtnW, tBtnH, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + 8 + tBtnW * 3, tBtnY, tBtnW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 8 + tBtnW * 3, tBtnY, tBtnH, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 8 + tBtnW * 3, tBtnY + tBtnH - 1, tBtnW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 7 + tBtnW * 4, tBtnY, tBtnH, DESK_BTN_DARK);
        gfx->setCursor(cx + 10 + tBtnW * 3, tBtnY + 2); gfx->print("S+");
        // Start/Pause and Reset buttons - 3D style
        int tBtn2Y = tBtnY + tBtnH + 2;
        int tBtn2W = w.maximized ? 50 : 38;
        int tBtn2H = w.maximized ? 14 : 12;
        uint16_t startCol = timerOn ? (timerPaused ? 0xFD20 : 0xAFE5) : DESK_BTN_FACE;
        gfx->fillRect(cx + 2, tBtn2Y, tBtn2W, tBtn2H, startCol);
        gfx->drawFastHLine(cx + 2, tBtn2Y, tBtn2W, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 2, tBtn2Y, tBtn2H, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 2, tBtn2Y + tBtn2H - 1, tBtn2W, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1 + tBtn2W, tBtn2Y, tBtn2H, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + 5, tBtn2Y + 3);
        gfx->print(timerOn ? (timerPaused ? "RESUME" : "PAUSE") : "START");
        // Reset button
        gfx->fillRect(cx + 4 + tBtn2W, tBtn2Y, tBtn2W - 8, tBtn2H, 0xFBE0);
        gfx->drawFastHLine(cx + 4 + tBtn2W, tBtn2Y, tBtn2W - 8, 0xFFFF);
        gfx->drawFastVLine(cx + 4 + tBtn2W, tBtn2Y, tBtn2H, 0xFFFF);
        gfx->drawFastHLine(cx + 4 + tBtn2W, tBtn2Y + tBtn2H - 1, tBtn2W - 8, 0xA000);
        gfx->drawFastVLine(cx + 3 + tBtn2W * 2 - 8, tBtn2Y, tBtn2H, 0xA000);
        gfx->setTextColor(0x8000);
        gfx->setCursor(cx + 7 + tBtn2W, tBtn2Y + 3);
        gfx->print("RESET");
      }
      break;
    case 3:  // TODO - Win98 style checkboxes with keyboard edit
      {
        if (deskTodoEditMode && deskTodoEditIdx >= 0) {
          // === EDIT MODE - show keyboard (Win98 style) ===
          int editDispH = w.maximized ? 16 : 12;
          // Sunken text field
          gfx->fillRect(cx + 1, cy + 1, cw - 2, editDispH, 0xFFFF);
          gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
          gfx->drawFastVLine(cx + 1, cy + 1, editDispH, DESK_BTN_DARK);
          gfx->setTextColor(0x0000);
          gfx->setCursor(cx + 3, cy + 3);
          char* editText = todoItems[deskTodoEditIdx];
          int editLen = strlen(editText);
          int maxDispChars = (cw - 6) / charW;
          int startChar = (deskTodoCursor > maxDispChars - 1) ? deskTodoCursor - maxDispChars + 1 : 0;
          for (int i = startChar; i < editLen && i < startChar + maxDispChars; i++) {
            if (i == deskTodoCursor) {
              gfx->setTextColor(0xFFFF);
              gfx->fillRect(gfx->getCursorX(), cy + 2, charW, editDispH - 2, DESK_TITLEBAR_COLOR);
            }
            gfx->print(editText[i]);
            if (i == deskTodoCursor) gfx->setTextColor(0x0000);
          }
          if (deskTodoCursor >= editLen) {
            gfx->fillRect(gfx->getCursorX(), cy + 2, charW, editDispH - 2, DESK_TITLEBAR_COLOR);
          }
          
          // Keyboard - Win98 3D buttons
          const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
          const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
          const char** rows = (kbMode == 1) ? rowsLower : rowsUpper;
          int nkw = w.maximized ? 18 : 11;
          int nkh = w.maximized ? 14 : 11;
          int ky = cy + editDispH + 2;
          int kbWidth = 10 * (nkw + 1) - 1;
          int kbOffsetX = cx + (cw - kbWidth) / 2;
          
          for (int r = 0; r < 4; r++) {
            int rowLen = strlen(rows[r]);
            int rowOffset = kbOffsetX + (10 - rowLen) * (nkw + 1) / 2;
            for (int c = 0; c < rowLen; c++) {
              int kx = rowOffset + c * (nkw + 1);
              // 3D key button
              gfx->fillRect(kx, ky, nkw, nkh, DESK_BTN_FACE);
              gfx->drawFastHLine(kx, ky, nkw, DESK_BTN_LIGHT);
              gfx->drawFastVLine(kx, ky, nkh, DESK_BTN_LIGHT);
              gfx->drawFastHLine(kx, ky + nkh - 1, nkw, DESK_BTN_DARK);
              gfx->drawFastVLine(kx + nkw - 1, ky, nkh, DESK_BTN_DARK);
              gfx->setTextColor(0x0000);
              gfx->setCursor(kx + (nkw - charW) / 2, ky + (nkh - 8) / 2);
              gfx->print(rows[r][c]);
            }
            ky += nkh + 1;
          }
          // Control buttons - Win98 3D style
          int btnW = w.maximized ? 28 : 20;
          int btnH = nkh;
          int btnY = ky;
          int btnX = kbOffsetX;
          // SPC button
          gfx->fillRect(btnX, btnY, btnW, btnH, DESK_BTN_FACE);
          gfx->drawFastHLine(btnX, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(btnX, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(btnX, btnY + btnH - 1, btnW, DESK_BTN_DARK);
          gfx->drawFastVLine(btnX + btnW - 1, btnY, btnH, DESK_BTN_DARK);
          gfx->setTextColor(0x0000); gfx->setCursor(btnX + 2, btnY + 2); gfx->print("SPC");
          btnX += btnW + 2;
          // DEL button
          gfx->fillRect(btnX, btnY, btnW, btnH, 0xFBE0);
          gfx->drawFastHLine(btnX, btnY, btnW, 0xFFFF);
          gfx->drawFastVLine(btnX, btnY, btnH, 0xFFFF);
          gfx->drawFastHLine(btnX, btnY + btnH - 1, btnW, 0xA000);
          gfx->drawFastVLine(btnX + btnW - 1, btnY, btnH, 0xA000);
          gfx->setTextColor(0x8000); gfx->setCursor(btnX + 2, btnY + 2); gfx->print("DEL");
          btnX += btnW + 2;
          // OK button
          gfx->fillRect(btnX, btnY, btnW, btnH, 0xAFE5);
          gfx->drawFastHLine(btnX, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(btnX, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(btnX, btnY + btnH - 1, btnW, 0x0400);
          gfx->drawFastVLine(btnX + btnW - 1, btnY, btnH, 0x0400);
          gfx->setTextColor(0x0000); gfx->setCursor(btnX + 4, btnY + 2); gfx->print("OK");
          btnX += btnW + 2;
          // Case button
          gfx->fillRect(btnX, btnY, btnW - 4, btnH, DESK_BTN_FACE);
          gfx->drawFastHLine(btnX, btnY, btnW - 4, DESK_BTN_LIGHT);
          gfx->drawFastVLine(btnX, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(btnX, btnY + btnH - 1, btnW - 4, DESK_BTN_DARK);
          gfx->drawFastVLine(btnX + btnW - 5, btnY, btnH, DESK_BTN_DARK);
          gfx->setTextColor(0x0000); gfx->setCursor(btnX + 2, btnY + 2); gfx->print(kbMode ? "AB" : "ab");
        } else {
          // === LIST MODE - Win98 style ===
          int itemH = w.maximized ? 14 : 10;
          int cbSize = w.maximized ? 10 : 8;
          int scrollW = 8;
          int btnH = w.maximized ? 14 : 10;
          int listH = ch - btnH - 4;
          int maxVisible = listH / itemH;
          
          for (int i = 0; i < maxVisible && (i + todoScrollOffset) < todoCount; i++) {
            int idx = i + todoScrollOffset;
            int iy = cy + 2 + i * itemH;
            if (iy + itemH > cy + listH) break;
            // Win98 style checkbox - sunken
            gfx->fillRect(cx + 2, iy, cbSize, cbSize, 0xFFFF);
            gfx->drawFastHLine(cx + 2, iy, cbSize, DESK_BTN_DARK);
            gfx->drawFastVLine(cx + 2, iy, cbSize, DESK_BTN_DARK);
            if (todoDone[idx]) {
              gfx->drawLine(cx + 3, iy + cbSize/2, cx + cbSize/2, iy + cbSize - 2, 0x0400);
              gfx->drawLine(cx + cbSize/2, iy + cbSize - 2, cx + cbSize, iy + 2, 0x0400);
            }
            // Item text - white on black background (gray if done)
            gfx->setTextColor(todoDone[idx] ? 0x8410 : 0xFFFF);
            gfx->setCursor(cx + cbSize + 6, iy);
            int maxChars = w.maximized ? 14 : 6;
            for (int j = 0; j < maxChars && todoItems[idx][j] != '\0'; j++) {
              gfx->print(todoItems[idx][j]);
            }
            // Delete button (X) - red
            int delX = cx + cw - scrollW - 12;
            gfx->setTextColor(0xF800);
            gfx->setCursor(delX, iy);
            gfx->print("X");
          }
          
          // Vertical scrollbar - Win98 style
          if (todoCount > maxVisible) {
            int sbX = cx + cw - scrollW - 1;
            int sbY = cy + 2;
            int sbH = listH - 4;
            gfx->fillRect(sbX, sbY, scrollW, sbH, DESK_BTN_FACE);
            gfx->drawFastHLine(sbX, sbY, scrollW, DESK_BTN_DARK);
            gfx->drawFastVLine(sbX, sbY, sbH, DESK_BTN_DARK);
            int thumbH = max(10, sbH * maxVisible / todoCount);
            int thumbY = sbY + (sbH - thumbH) * todoScrollOffset / max(1, todoCount - maxVisible);
            // 3D thumb
            gfx->fillRect(sbX + 1, thumbY, scrollW - 2, thumbH, DESK_BTN_FACE);
            gfx->drawFastHLine(sbX + 1, thumbY, scrollW - 2, DESK_BTN_LIGHT);
            gfx->drawFastVLine(sbX + 1, thumbY, thumbH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(sbX + 1, thumbY + thumbH - 1, scrollW - 2, DESK_BTN_DARK);
            gfx->drawFastVLine(sbX + scrollW - 2, thumbY, thumbH, DESK_BTN_DARK);
          }
          
          // Add button at bottom - Win98 3D style
          if (todoCount < MAX_TODO) {
            int addY = cy + listH + 2;
            gfx->fillRect(cx + 2, addY, cw - 4, btnH, 0xAFE5);
            gfx->drawFastHLine(cx + 2, addY, cw - 4, DESK_BTN_LIGHT);
            gfx->drawFastVLine(cx + 2, addY, btnH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(cx + 2, addY + btnH - 1, cw - 4, 0x0400);
            gfx->drawFastVLine(cx + cw - 3, addY, btnH, 0x0400);
            gfx->setTextColor(0x0000);
            gfx->setCursor(cx + cw/2 - 12, addY + 1);
            gfx->print("+ ADD");
          }
        }
      }
      break;
    case 4:  // Notes - Win98 style with SAVE button
      {
        // Determine if we should use large keyboard (fullscreen or maximized)
        bool largeKB = w.fullscreen || w.maximized;
        
        // Note display area - sunken 3D style
        int noteDispH = largeKB ? 20 : 16;
        gfx->fillRect(cx + 1, cy + 1, cw - 2, noteDispH, 0xFFFF);
        gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1, cy + 1, noteDispH, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + 3, cy + 4);
        // Show text with cursor indicator at position
        int maxChars = (cw - 6) / charW;
        int noteLen = strlen(notesText);
        if (notesCursor > noteLen) notesCursor = noteLen;
        int startChar = (notesCursor > maxChars - 1) ? notesCursor - maxChars + 1 : 0;
        for (int i = startChar; i < noteLen && i < startChar + maxChars; i++) {
          if (i == notesCursor) {
            gfx->setTextColor(0xFFFF);
            gfx->fillRect(gfx->getCursorX(), cy + 2, charW, noteDispH - 2, DESK_TITLEBAR_COLOR);
          }
          gfx->print(notesText[i]);
          if (i == notesCursor) gfx->setTextColor(0x0000);
        }
        if (notesCursor >= noteLen) {
          gfx->fillRect(gfx->getCursorX(), cy + 2, charW, noteDispH - 2, DESK_TITLEBAR_COLOR);
        }
        
        // Keyboard - Win98 3D buttons (larger when fullscreen)
        const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
        const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
        const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
        const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
        int nkw = largeKB ? 22 : 11;  // Larger keys when fullscreen
        int nkh = largeKB ? 18 : 11;
        int nky = cy + noteDispH + 2;
        int kbWidth = 10 * (nkw + 1) - 1;
        int kbOffsetX = largeKB ? (cw - kbWidth) / 2 : 1;
        
        // Rows 1-2 (10 keys each) - 3D buttons
        for (int r = 0; r < 2; r++) {
          for (int i = 0; i < 10; i++) {
            int kx = cx + kbOffsetX + i * (nkw + 1);
            gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
            gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
            gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
            gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
            gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
            gfx->print(rows[r][i]);
          }
          nky += nkh + 1;
        }
        // Row 3 - 3D buttons
        int row3Len = (kbMode == 2) ? 10 : 9;
        int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
        for (int i = 0; i < row3Len; i++) {
          int kx = cx + row3Offset + i * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
          gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
          gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
          gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
          gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
          gfx->setTextColor(0x0000);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[2][i]);
        }
        nky += nkh + 1;
        // Row 4: mode buttons - 3D style
        int modeW = nkw;
        // aA button
        uint16_t aaCol = (kbMode < 2) ? 0xAFE5 : DESK_BTN_FACE;
        gfx->fillRect(cx + kbOffsetX, nky, modeW, nkh, aaCol);
        gfx->drawFastHLine(cx + kbOffsetX, nky, modeW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX, nky + nkh - 1, modeW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + modeW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + kbOffsetX + 2, nky + (nkh - 8)/2);
        gfx->print(kbMode == 1 ? "aA" : "Aa");
        // #! button
        uint16_t symCol = (kbMode == 2) ? 0xAFE5 : DESK_BTN_FACE;
        gfx->fillRect(cx + kbOffsetX + modeW + 1, nky, modeW, nkh, symCol);
        gfx->drawFastHLine(cx + kbOffsetX + modeW + 1, nky, modeW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX + modeW + 1, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX + modeW + 1, nky + nkh - 1, modeW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + modeW * 2, nky, nkh, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + kbOffsetX + modeW + 3, nky + (nkh - 8)/2);
        gfx->print("#!");
        // Letters/symbols - 3D buttons
        int row4Len = (kbMode == 2) ? 2 : 7;
        int row4Start = 2;
        for (int i = 0; i < row4Len; i++) {
          int kx = cx + kbOffsetX + (row4Start + i) * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
          gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
          gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
          gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
          gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
          gfx->setTextColor(0x0000);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[3][i]);
        }
        // DEL button - 3D red
        int delX = cx + kbOffsetX + 9 * (nkw + 1);
        gfx->fillRect(delX, nky, nkw, nkh, 0xFBE0);
        gfx->drawFastHLine(delX, nky, nkw, 0xFFFF);
        gfx->drawFastVLine(delX, nky, nkh, 0xFFFF);
        gfx->drawFastHLine(delX, nky + nkh - 1, nkw, 0xA000);
        gfx->drawFastVLine(delX + nkw - 1, nky, nkh, 0xA000);
        gfx->setTextColor(0x8000);
        gfx->setCursor(delX + nkw/2 - 2, nky + (nkh - 8)/2);
        gfx->print("X");
        nky += nkh + 1;
        // Row 5: Arrow keys + SPACE + CLR + SAVE - all 3D style
        int arrowW = nkw;
        // Left arrow - 3D
        gfx->fillRect(cx + kbOffsetX, nky, arrowW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + kbOffsetX, nky, arrowW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX, nky + nkh - 1, arrowW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + kbOffsetX + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print("<");
        // Right arrow - 3D
        gfx->fillRect(cx + kbOffsetX + arrowW + 1, nky, arrowW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + kbOffsetX + arrowW + 1, nky, arrowW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW + 1, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX + arrowW + 1, nky + nkh - 1, arrowW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW * 2, nky, nkh, DESK_BTN_DARK);
        gfx->setCursor(cx + kbOffsetX + arrowW + 1 + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print(">");
        // SPACE - 3D
        int spaceStartX = cx + kbOffsetX + 2 * (arrowW + 1);
        int spaceW = largeKB ? nkw * 3 : nkw * 2 + 3;
        gfx->fillRect(spaceStartX, nky, spaceW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(spaceStartX, nky, spaceW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(spaceStartX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(spaceStartX, nky + nkh - 1, spaceW, DESK_BTN_DARK);
        gfx->drawFastVLine(spaceStartX + spaceW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setCursor(spaceStartX + spaceW/2 - 8, nky + (nkh - 8)/2);
        gfx->print("SP");
        // CLR button - 3D orange
        int clrW = largeKB ? nkw + 8 : nkw * 2;
        int clrX = spaceStartX + spaceW + 2;
        gfx->fillRect(clrX, nky, clrW, nkh, 0xFD20);
        gfx->drawFastHLine(clrX, nky, clrW, 0xFFFF);
        gfx->drawFastVLine(clrX, nky, nkh, 0xFFFF);
        gfx->drawFastHLine(clrX, nky + nkh - 1, clrW, 0xC400);
        gfx->drawFastVLine(clrX + clrW - 1, nky, nkh, 0xC400);
        gfx->setCursor(clrX + 2, nky + (nkh - 8)/2);
        gfx->print("C");
        // SAVE button - 3D green
        if (sdCardMounted) {
          int saveW = largeKB ? nkw * 2 + 8 : nkw * 2 + 4;
          int saveX = clrX + clrW + 2;
          gfx->fillRect(saveX, nky, saveW, nkh, 0xAFE5);
          gfx->drawFastHLine(saveX, nky, saveW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(saveX, nky, nkh, DESK_BTN_LIGHT);
          gfx->drawFastHLine(saveX, nky + nkh - 1, saveW, 0x0400);
          gfx->drawFastVLine(saveX + saveW - 1, nky, nkh, 0x0400);
          gfx->setCursor(saveX + 2, nky + (nkh - 8)/2);
          gfx->print("SAV");
        }
      }
      break;
    case 7:  // Editor - Win98 style for editing .txt files
      {
        // File path - sunken 3D style
        int noteDispH = w.maximized ? 20 : 16;
        gfx->fillRect(cx + 1, cy + 1, cw - 2, noteDispH, 0xFFFF);
        gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + 1, cy + 1, noteDispH, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + 3, cy + 4);
        int maxChars = (cw - 6) / charW;
        int edLen = strlen(editorText);
        if (editorCursor > edLen) editorCursor = edLen;
        int startChar = (editorCursor > maxChars - 1) ? editorCursor - maxChars + 1 : 0;
        for (int i = startChar; i < edLen && i < startChar + maxChars; i++) {
          if (i == editorCursor) {
            gfx->setTextColor(0xFFFF);
            gfx->fillRect(gfx->getCursorX(), cy + 2, charW, noteDispH - 2, DESK_TITLEBAR_COLOR);
          }
          gfx->print(editorText[i]);
          if (i == editorCursor) gfx->setTextColor(0x0000);
        }
        if (editorCursor >= edLen) {
          gfx->fillRect(gfx->getCursorX(), cy + 2, charW, noteDispH - 2, DESK_TITLEBAR_COLOR);
        }
        
        // Keyboard - Win98 3D buttons
        const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
        const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
        const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
        const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
        int nkw = w.maximized ? 18 : 11;
        int nkh = w.maximized ? 14 : 11;
        int nky = cy + noteDispH + 2;
        int kbWidth = 10 * (nkw + 1) - 1;
        int kbOffsetX = w.maximized ? (cw - kbWidth) / 2 : 1;
        
        // Rows 1-2 - 3D buttons
        for (int r = 0; r < 2; r++) {
          for (int i = 0; i < 10; i++) {
            int kx = cx + kbOffsetX + i * (nkw + 1);
            gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
            gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
            gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
            gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
            gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
            gfx->setTextColor(0x0000);
            gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
            gfx->print(rows[r][i]);
          }
          nky += nkh + 1;
        }
        // Row 3 - 3D buttons
        int row3Len = (kbMode == 2) ? 10 : 9;
        int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
        for (int i = 0; i < row3Len; i++) {
          int kx = cx + row3Offset + i * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
          gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
          gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
          gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
          gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
          gfx->setTextColor(0x0000);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[2][i]);
        }
        nky += nkh + 1;
        // Row 4: mode buttons - 3D style
        int modeW = nkw;
        uint16_t aaCol = (kbMode < 2) ? 0xAFE5 : DESK_BTN_FACE;
        gfx->fillRect(cx + kbOffsetX, nky, modeW, nkh, aaCol);
        gfx->drawFastHLine(cx + kbOffsetX, nky, modeW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX, nky + nkh - 1, modeW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + modeW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + kbOffsetX + 2, nky + (nkh - 8)/2);
        gfx->print(kbMode == 1 ? "aA" : "Aa");
        // #! button - 3D
        uint16_t symCol = (kbMode == 2) ? 0xAFE5 : DESK_BTN_FACE;
        gfx->fillRect(cx + kbOffsetX + modeW + 1, nky, modeW, nkh, symCol);
        gfx->drawFastHLine(cx + kbOffsetX + modeW + 1, nky, modeW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX + modeW + 1, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX + modeW + 1, nky + nkh - 1, modeW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + modeW * 2, nky, nkh, DESK_BTN_DARK);
        gfx->setCursor(cx + kbOffsetX + modeW + 3, nky + (nkh - 8)/2);
        gfx->print("#!");
        // Letters/symbols - 3D buttons
        int row4Len = (kbMode == 2) ? 2 : 7;
        int row4Start = 2;
        for (int i = 0; i < row4Len; i++) {
          int kx = cx + kbOffsetX + (row4Start + i) * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, DESK_BTN_FACE);
          gfx->drawFastHLine(kx, nky, nkw, DESK_BTN_LIGHT);
          gfx->drawFastVLine(kx, nky, nkh, DESK_BTN_LIGHT);
          gfx->drawFastHLine(kx, nky + nkh - 1, nkw, DESK_BTN_DARK);
          gfx->drawFastVLine(kx + nkw - 1, nky, nkh, DESK_BTN_DARK);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[3][i]);
        }
        // DEL button - 3D red
        int delX = cx + kbOffsetX + 9 * (nkw + 1);
        gfx->fillRect(delX, nky, nkw, nkh, 0xFBE0);
        gfx->drawFastHLine(delX, nky, nkw, 0xFFFF);
        gfx->drawFastVLine(delX, nky, nkh, 0xFFFF);
        gfx->drawFastHLine(delX, nky + nkh - 1, nkw, 0xA000);
        gfx->drawFastVLine(delX + nkw - 1, nky, nkh, 0xA000);
        gfx->setTextColor(0x8000);
        gfx->setCursor(delX + nkw/2 - 2, nky + (nkh - 8)/2);
        gfx->print("X");
        nky += nkh + 1;
        // Row 5: Arrow keys + SPACE + CANCEL + SAVE - 3D style
        int arrowW = nkw;
        // Left arrow
        gfx->fillRect(cx + kbOffsetX, nky, arrowW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + kbOffsetX, nky, arrowW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX, nky + nkh - 1, arrowW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + kbOffsetX + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print("<");
        // Right arrow
        gfx->fillRect(cx + kbOffsetX + arrowW + 1, nky, arrowW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(cx + kbOffsetX + arrowW + 1, nky, arrowW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW + 1, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + kbOffsetX + arrowW + 1, nky + nkh - 1, arrowW, DESK_BTN_DARK);
        gfx->drawFastVLine(cx + kbOffsetX + arrowW * 2, nky, nkh, DESK_BTN_DARK);
        gfx->setCursor(cx + kbOffsetX + arrowW + 1 + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print(">");
        // SPACE
        int spaceStartX = cx + kbOffsetX + 2 * (arrowW + 1);
        int spaceW = w.maximized ? nkw * 2 : nkw * 2;
        gfx->fillRect(spaceStartX, nky, spaceW, nkh, DESK_BTN_FACE);
        gfx->drawFastHLine(spaceStartX, nky, spaceW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(spaceStartX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(spaceStartX, nky + nkh - 1, spaceW, DESK_BTN_DARK);
        gfx->drawFastVLine(spaceStartX + spaceW - 1, nky, nkh, DESK_BTN_DARK);
        gfx->setCursor(spaceStartX + spaceW/2 - 8, nky + (nkh - 8)/2);
        gfx->print("SP");
        // CANCEL button - 3D red
        int cancelW = w.maximized ? nkw * 2 + 4 : nkw * 2 + 2;
        int cancelX = spaceStartX + spaceW + 2;
        gfx->fillRect(cancelX, nky, cancelW, nkh, 0xFBE0);
        gfx->drawFastHLine(cancelX, nky, cancelW, 0xFFFF);
        gfx->drawFastVLine(cancelX, nky, nkh, 0xFFFF);
        gfx->drawFastHLine(cancelX, nky + nkh - 1, cancelW, 0xA000);
        gfx->drawFastVLine(cancelX + cancelW - 1, nky, nkh, 0xA000);
        gfx->setTextColor(0x8000);
        gfx->setCursor(cancelX + 2, nky + (nkh - 8)/2);
        gfx->print("CAN");
        // SAVE button - 3D green
        int saveW = w.maximized ? nkw * 2 + 4 : nkw * 2 + 2;
        int saveX = cancelX + cancelW + 2;
        gfx->fillRect(saveX, nky, saveW, nkh, 0xAFE5);
        gfx->drawFastHLine(saveX, nky, saveW, DESK_BTN_LIGHT);
        gfx->drawFastVLine(saveX, nky, nkh, DESK_BTN_LIGHT);
        gfx->drawFastHLine(saveX, nky + nkh - 1, saveW, 0x0400);
        gfx->drawFastVLine(saveX + saveW - 1, nky, nkh, 0x0400);
        gfx->setTextColor(0x0000);
        gfx->setCursor(saveX + 2, nky + (nkh - 8)/2);
        gfx->print("SAV");
        gfx->setTextColor(COLOR_TEXT);
      }
      break;
    case 8:  // WiFi - dropdown network list and password entry
      {
        int fieldH = w.maximized ? 16 : 14;
        int scrollW = 6;
        
        // SSID dropdown at top
        gfx->fillRect(cx + 1, cy + 1, cw - 2, fieldH, COLOR_HIGHLIGHT);
        gfx->drawRect(cx + 1, cy + 1, cw - 2, fieldH, COLOR_ACCENT);
        gfx->setTextColor(COLOR_TEXT);
        gfx->setCursor(cx + 3, cy + 4);
        if (strlen(wifiEditSSID) > 0) {
          gfx->print(wifiEditSSID);
        } else {
          gfx->setTextColor(0x6B6D);
          gfx->print("Select Network");
        }
        // Dropdown arrow
        int arrowX = cx + cw - 12;
        gfx->setTextColor(COLOR_ACCENT);
        gfx->setCursor(arrowX, cy + 4);
        gfx->print(wifiDropdownOpen ? "^" : "v");
        
        // Password field below
        int passY = cy + fieldH + 2;
        gfx->drawRect(cx + 1, passY, cw - 2, fieldH, COLOR_HIGHLIGHT);
        gfx->setTextColor(COLOR_TEXT);
        gfx->setCursor(cx + 3, passY + 3);
        int maxChars = (cw - 6) / charW;
        int passLen = strlen(wifiEditPass);
        if (wifiEditCursor > passLen) wifiEditCursor = passLen;
        int startChar = (wifiEditCursor > maxChars - 1) ? wifiEditCursor - maxChars + 1 : 0;
        for (int i = startChar; i < passLen && i < startChar + maxChars; i++) {
          if (i == wifiEditCursor) gfx->print("|");
          gfx->print(wifiEditPass[i]);
        }
        if (wifiEditCursor >= passLen) gfx->print("_");
        
        // Keyboard layout - kbMode: 0=uppercase, 1=lowercase, 2=symbols
        const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
        const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
        const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
        const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
        int nkw = w.maximized ? 14 : 11;
        int nkh = w.maximized ? 12 : 11;
        int nky = passY + fieldH + 2;
        int kbWidth = 10 * (nkw + 1) - 1;
        int kbOffsetX = (cw - kbWidth) / 2;
        
        // Rows 1-2 (10 keys each)
        for (int r = 0; r < 2; r++) {
          for (int i = 0; i < 10; i++) {
            int kx = cx + kbOffsetX + i * (nkw + 1);
            gfx->fillRect(kx, nky, nkw, nkh, COLOR_BG);
            gfx->drawRect(kx, nky, nkw, nkh, COLOR_HIGHLIGHT);
            gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
            gfx->print(rows[r][i]);
          }
          nky += nkh + 1;
        }
        // Row 3 (9 or 10 keys depending on mode)
        int row3Len = (kbMode == 2) ? 10 : 9;
        int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
        for (int i = 0; i < row3Len; i++) {
          int kx = cx + row3Offset + i * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, COLOR_BG);
          gfx->drawRect(kx, nky, nkw, nkh, COLOR_HIGHLIGHT);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[2][i]);
        }
        nky += nkh + 1;
        // Row 4: mode buttons + letters/symbols + DEL + arrows
        // aA button (toggle case)
        int modeW = nkw;
        gfx->fillRect(cx + kbOffsetX, nky, modeW, nkh, (kbMode < 2) ? 0x07E0 : 0x4208);
        gfx->drawRect(cx + kbOffsetX, nky, modeW, nkh, 0xFFFF);
        gfx->setTextColor((kbMode < 2) ? 0x0000 : 0xFFFF);
        gfx->setCursor(cx + kbOffsetX + 2, nky + (nkh - 8)/2);
        gfx->print(kbMode == 1 ? "aA" : "Aa");
        // #! button (symbols)
        gfx->fillRect(cx + kbOffsetX + modeW + 1, nky, modeW, nkh, (kbMode == 2) ? 0x07E0 : 0x4208);
        gfx->drawRect(cx + kbOffsetX + modeW + 1, nky, modeW, nkh, 0xFFFF);
        gfx->setTextColor((kbMode == 2) ? 0x0000 : 0xFFFF);
        gfx->setCursor(cx + kbOffsetX + modeW + 3, nky + (nkh - 8)/2);
        gfx->print("#!");
        gfx->setTextColor(COLOR_TEXT);
        // Letters/symbols (7 keys for letters, 2 for symbols)
        int row4Len = (kbMode == 2) ? 2 : 7;
        int row4Start = 2;
        for (int i = 0; i < row4Len; i++) {
          int kx = cx + kbOffsetX + (row4Start + i) * (nkw + 1);
          gfx->fillRect(kx, nky, nkw, nkh, COLOR_BG);
          gfx->drawRect(kx, nky, nkw, nkh, COLOR_HIGHLIGHT);
          gfx->setCursor(kx + nkw/2 - 2, nky + (nkh - 8)/2);
          gfx->print(rows[3][i]);
        }
        // DEL button (position 9)
        int delX = cx + kbOffsetX + 9 * (nkw + 1);
        gfx->fillRect(delX, nky, nkw, nkh, 0xF800);
        gfx->drawRect(delX, nky, nkw, nkh, 0xFFFF);
        gfx->setTextColor(0xFFFF);
        gfx->setCursor(delX + nkw/2 - 2, nky + (nkh - 8)/2);
        gfx->print("X");
        gfx->setTextColor(COLOR_TEXT);
        nky += nkh + 1;
        // Row 5: Arrow keys + SPACE + X (cancel) + OK (connect)
        // Left arrow
        int arrowW = nkw;
        gfx->fillRect(cx + kbOffsetX, nky, arrowW, nkh, 0x4208);
        gfx->drawRect(cx + kbOffsetX, nky, arrowW, nkh, 0xFFFF);
        gfx->setTextColor(0xFFFF);
        gfx->setCursor(cx + kbOffsetX + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print("<");
        // Right arrow
        gfx->fillRect(cx + kbOffsetX + arrowW + 1, nky, arrowW, nkh, 0x4208);
        gfx->drawRect(cx + kbOffsetX + arrowW + 1, nky, arrowW, nkh, 0xFFFF);
        gfx->setCursor(cx + kbOffsetX + arrowW + 1 + arrowW/2 - 2, nky + (nkh - 8)/2);
        gfx->print(">");
        gfx->setTextColor(COLOR_TEXT);
        // SPACE (after arrows)
        int spaceStartX = cx + kbOffsetX + 2 * (arrowW + 1);
        int spaceW = nkw * 2;
        gfx->fillRect(spaceStartX, nky, spaceW, nkh, COLOR_BG);
        gfx->drawRect(spaceStartX, nky, spaceW, nkh, COLOR_HIGHLIGHT);
        gfx->setCursor(spaceStartX + spaceW/2 - 8, nky + (nkh - 8)/2);
        gfx->print("SP");
        // X button (cancel - red)
        int cancelW = nkw * 2 + 2;
        int cancelX = spaceStartX + spaceW + 2;
        gfx->fillRect(cancelX, nky, cancelW, nkh, 0xF800);
        gfx->drawRect(cancelX, nky, cancelW, nkh, 0xFFFF);
        gfx->setTextColor(0xFFFF);
        gfx->setCursor(cancelX + cancelW/2 - 2, nky + (nkh - 8)/2);
        gfx->print("X");
        // OK button (connect - green)
        int connectW = nkw * 2 + 2;
        int connectX = cancelX + cancelW + 2;
        gfx->fillRect(connectX, nky, connectW, nkh, 0x07E0);
        gfx->drawRect(connectX, nky, connectW, nkh, 0xFFFF);
        gfx->setTextColor(0x0000);
        gfx->setCursor(connectX + 2, nky + (nkh - 8)/2);
        gfx->print("OK");
        gfx->setTextColor(COLOR_TEXT);
        
        // Dropdown overlay (drawn last, on top of everything)
        if (wifiDropdownOpen) {
          int dropY = cy + fieldH + 1;
          int dropH = min(wifiScanCount, 4) * fieldH + 2;
          int maxVisible = 4;
          gfx->fillRect(cx + 1, dropY, cw - 2, dropH, COLOR_BG);
          gfx->drawRect(cx + 1, dropY, cw - 2, dropH, COLOR_ACCENT);
          // Network list
          for (int i = 0; i < maxVisible && (i + wifiDropdownScroll) < wifiScanCount; i++) {
            int idx = i + wifiDropdownScroll;
            int iy = dropY + 1 + i * fieldH;
            bool sel = (idx == wifiScanSel);
            if (sel) gfx->fillRect(cx + 2, iy, cw - scrollW - 5, fieldH - 1, COLOR_HIGHLIGHT);
            gfx->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
            gfx->setCursor(cx + 4, iy + 3);
            // Truncate long SSIDs
            for (int j = 0; j < 16 && wifiNetworks[idx][j] != '\0'; j++) {
              gfx->print(wifiNetworks[idx][j]);
            }
          }
          // Scrollbar if needed
          if (wifiScanCount > maxVisible) {
            int sbX = cx + cw - scrollW - 2;
            int sbY = dropY + 1;
            int sbH = dropH - 2;
            gfx->fillRect(sbX, sbY, scrollW, sbH, 0x4208);
            int thumbH = max(8, sbH * maxVisible / wifiScanCount);
            int thumbY = sbY + (sbH - thumbH) * wifiDropdownScroll / max(1, wifiScanCount - maxVisible);
            gfx->fillRect(sbX + 1, thumbY, scrollW - 2, thumbH, COLOR_ACCENT);
          }
        }
      }
      break;
    case 9:  // Tetris game
      {
        // Larger cell size for fullscreen, centered board
        int cellSize = 6;  // Bigger cells for fullscreen
        int boardW = TETRIS_COLS * cellSize;
        int boardH = TETRIS_ROWS * cellSize;
        // Center the board horizontally with space for info on right
        int boardX = cx + (cw - boardW - 50) / 2;  // Leave 50px for info panel
        int boardY = cy + (ch - boardH) / 2;  // Center vertically
        int infoX = boardX + boardW + 8;
        
        // Require fullscreen mode
        if (!w.fullscreen) {
          gfx->setTextColor(COLOR_ACCENT);
          gfx->setCursor(cx + 4, cy + ch/2 - 20);
          gfx->print("Please be in");
          gfx->setCursor(cx + 4, cy + ch/2 - 8);
          gfx->print("fullscreen to");
          gfx->setCursor(cx + 4, cy + ch/2 + 4);
          gfx->print("use this app.");
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 4, cy + ch/2 + 20);
          gfx->print("Press SET");
        } else if (tetrisShowMenu && !tetrisPlaying) {
          // Main menu screen (fullscreen)
          gfx->setTextColor(0x07FF);  // Cyan
          gfx->setCursor(cx + cw/2 - 18, cy + 20);
          gfx->print("TETRIS");
          
          // Draw decorative blocks
          for (int i = 0; i < 7; i++) {
            int bx = cx + cw/2 - 42 + i * 12;
            gfx->fillRect(bx, cy + 35, 10, 10, tetrisColors[i]);
          }
          
          // Start button (highlighted if selected)
          int btnW = 60, btnH = 18;
          int btnX = cx + cw/2 - btnW/2;
          int btnY = cy + 55;
          uint16_t btnColor = (tetrisMenuSel == 0) ? 0x07E0 : 0x4208;  // Green if selected
          gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 12, btnY + 5);
          gfx->print("START");
          
          // Controls info
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 10, cy + 85);
          gfx->print("L/R = Move piece");
          gfx->setCursor(cx + 10, cy + 97);
          gfx->print("UP  = Rotate");
          gfx->setCursor(cx + 10, cy + 109);
          gfx->print("DN  = Fast drop");
          gfx->setCursor(cx + 10, cy + 121);
          gfx->print("MID = Pause/Menu");
        } else if (tetrisGameOver) {
          // Game over screen (fullscreen)
          gfx->setTextColor(0xF800);  // Red
          gfx->setCursor(cx + cw/2 - 30, cy + 20);
          gfx->print("GAME OVER");
          
          // Final score
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + cw/2 - 30, cy + 45);
          gfx->print("Score: ");
          gfx->print(tetrisScore);
          
          gfx->setCursor(cx + cw/2 - 30, cy + 60);
          gfx->print("Lines: ");
          gfx->print(tetrisLines);
          
          // Retry button
          int btnW = 60, btnH = 18;
          int btnX = cx + cw/2 - btnW/2;
          int btnY = cy + 80;
          uint16_t btnColor = (tetrisMenuSel == 0) ? 0x07E0 : 0x4208;
          gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 12, btnY + 5);
          gfx->print("RETRY");
          
          // Menu button
          btnY = cy + 105;
          btnColor = (tetrisMenuSel == 1) ? 0x07E0 : 0x4208;
          gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 15, btnY + 5);
          gfx->print("MENU");
        } else if (tetrisPlaying) {
          // Draw board border
          gfx->drawRect(boardX - 1, boardY - 1, boardW + 2, boardH + 2, COLOR_HIGHLIGHT);
          
          // Draw board cells
          for (int r = 0; r < TETRIS_ROWS; r++) {
            for (int c = 0; c < TETRIS_COLS; c++) {
              int px = boardX + c * cellSize;
              int py = boardY + r * cellSize;
              if (tetrisBoard[r][c] > 0) {
                gfx->fillRect(px, py, cellSize - 1, cellSize - 1, tetrisColors[tetrisBoard[r][c] - 1]);
              }
            }
          }
          
          // Draw current piece
          uint16_t piece = tetrisPieces[tetrisPieceType][tetrisPieceRot];
          for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
              if (piece & (0x8000 >> (r * 4 + c))) {
                int br = tetrisPieceY + r;
                int bc = tetrisPieceX + c;
                if (br >= 0 && br < TETRIS_ROWS && bc >= 0 && bc < TETRIS_COLS) {
                  int px = boardX + bc * cellSize;
                  int py = boardY + br * cellSize;
                  gfx->fillRect(px, py, cellSize - 1, cellSize - 1, tetrisColors[tetrisPieceType]);
                }
              }
            }
          }
          
          // Draw score/level info
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(infoX, boardY);
          gfx->print("Score");
          gfx->setCursor(infoX, boardY + 10);
          gfx->print(tetrisScore);
          gfx->setCursor(infoX, boardY + 28);
          gfx->print("Lines");
          gfx->setCursor(infoX, boardY + 38);
          gfx->print(tetrisLines);
          
          // Draw pause overlay if paused
          if (tetrisPaused) {
            // Semi-transparent overlay effect (dark box)
            gfx->fillRect(cx + cw/2 - 50, cy + ch/2 - 40, 100, 80, 0x0000);
            gfx->drawRect(cx + cw/2 - 50, cy + ch/2 - 40, 100, 80, 0xFFFF);
            
            gfx->setTextColor(0xFFE0);  // Yellow
            gfx->setCursor(cx + cw/2 - 21, cy + ch/2 - 30);
            gfx->print("PAUSED");
            
            // Resume button
            int btnW = 70, btnH = 18;
            int btnX = cx + cw/2 - btnW/2;
            int btnY = cy + ch/2 - 10;
            uint16_t btnColor = (tetrisMenuSel == 0) ? 0x07E0 : 0x4208;
            gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
            gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
            gfx->setTextColor(0xFFFF);
            gfx->setCursor(btnX + 14, btnY + 5);
            gfx->print("RESUME");
            
            // Menu button
            btnY = cy + ch/2 + 15;
            btnColor = (tetrisMenuSel == 1) ? 0x07E0 : 0x4208;
            gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
            gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
            gfx->setTextColor(0xFFFF);
            gfx->setCursor(btnX + 20, btnY + 5);
            gfx->print("MENU");
          }
        }
      }
      break;
    case 10:  // Paint app - requires fullscreen, uses mouse
      {
        // Require fullscreen mode
        if (!w.fullscreen) {
          gfx->setTextColor(COLOR_ACCENT);
          gfx->setCursor(cx + 4, cy + ch/2 - 20);
          gfx->print("Please be in");
          gfx->setCursor(cx + 4, cy + ch/2 - 8);
          gfx->print("fullscreen to");
          gfx->setCursor(cx + 4, cy + ch/2 + 4);
          gfx->print("use this app.");
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 4, cy + ch/2 + 20);
          gfx->print("Press SET");
        } else {
          int sidebarW = 20;
          int canvasAreaW = cw - sidebarW - 4;
          int canvasAreaH = ch - 4;
          
          // Calculate cell size to fit canvas in available area
          int cellSize = min(canvasAreaW / PAINT_WIDTH, canvasAreaH / PAINT_HEIGHT);
          if (cellSize < 1) cellSize = 1;
          int canvasW = PAINT_WIDTH * cellSize;
          int canvasH = PAINT_HEIGHT * cellSize;
          int canvasX = cx + 2;
          int canvasY = cy + 2;
          
          // Draw canvas border
          gfx->drawRect(canvasX - 1, canvasY - 1, canvasW + 2, canvasH + 2, COLOR_TEXT);
          
          // Draw canvas pixels
          for (int py = 0; py < PAINT_HEIGHT; py++) {
            for (int px = 0; px < PAINT_WIDTH; px++) {
              gfx->fillRect(canvasX + px * cellSize, canvasY + py * cellSize, cellSize, cellSize, paintCanvas[py][px]);
            }
          }
          
          // Draw sidebar on right
          int sbX = cx + cw - sidebarW;
          gfx->fillRect(sbX, cy, sidebarW, ch, 0x4208);
          gfx->drawLine(sbX, cy, sbX, cy + ch, COLOR_TEXT);
          
          // Brush button (top)
          uint16_t brushBtnColor = paintEraser ? 0x4208 : 0x07E0;
          gfx->fillRect(sbX + 2, cy + 2, 16, 14, brushBtnColor);
          gfx->drawRect(sbX + 2, cy + 2, 16, 14, COLOR_TEXT);
          gfx->setTextColor(0x0000);
          gfx->setCursor(sbX + 5, cy + 5);
          gfx->print("B");
          
          // Eraser button
          uint16_t eraserBtnColor = paintEraser ? 0xF800 : 0x4208;
          gfx->fillRect(sbX + 2, cy + 18, 16, 14, eraserBtnColor);
          gfx->drawRect(sbX + 2, cy + 18, 16, 14, COLOR_TEXT);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(sbX + 5, cy + 21);
          gfx->print("E");
          
          // Color palette (10 colors in 2 columns)
          for (int i = 0; i < 10; i++) {
            int col = i % 2;
            int row = i / 2;
            int palX = sbX + 2 + col * 9;
            int palY = cy + 36 + row * 10;
            gfx->fillRect(palX, palY, 8, 8, paintPalette[i]);
            if (i == paintColorIdx && !paintEraser) {
              gfx->drawRect(palX - 1, palY - 1, 10, 10, COLOR_ACCENT);
            }
          }
          
          // Save button at bottom
          gfx->fillRect(sbX + 2, cy + ch - 14, 16, 12, 0x001F);
          gfx->drawRect(sbX + 2, cy + ch - 14, 16, 12, COLOR_TEXT);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(sbX + 5, cy + ch - 11);
          gfx->print("S");
          
          // Check mouse clicks on canvas
          if (mouseX >= canvasX && mouseX < canvasX + canvasW &&
              mouseY >= canvasY && mouseY < canvasY + canvasH) {
            int px = (mouseX - canvasX) / cellSize;
            int py = (mouseY - canvasY) / cellSize;
            if (px >= 0 && px < PAINT_WIDTH && py >= 0 && py < PAINT_HEIGHT) {
              // Draw cursor highlight at mouse position
              gfx->drawRect(canvasX + px * cellSize, canvasY + py * cellSize, cellSize, cellSize, COLOR_ACCENT);
            }
          }
        }
      }
      break;
    case 12:  // Image viewer (BMP) - fullscreen with auto-hiding close button
      {
        if (!viewingBMP) {
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 4, cy + ch/2 - 4);
          gfx->print("No image loaded");
        } else if (bmpViewBuffer && bmpViewWidth > 0 && bmpViewHeight > 0) {
          // Copy BMP from bmpViewBuffer to gfx buffer using memcpy for speed
          uint16_t* destBuf = deskBuffer->getBuffer();
          memcpy(destBuf, bmpViewBuffer, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
          
          // Check if cursor moved - show overlay
          if (mouseX != imgViewerLastCursorX || mouseY != imgViewerLastCursorY) {
            imgViewerOverlayVisible = true;
            imgViewerOverlayTime = millis();
            imgViewerLastCursorX = mouseX;
            imgViewerLastCursorY = mouseY;
          }
          
          // Hide overlay after 5 seconds
          if (imgViewerOverlayVisible && millis() - imgViewerOverlayTime > 5000) {
            imgViewerOverlayVisible = false;
          }
          
          // Draw close button overlay in top-right corner if visible
          if (imgViewerOverlayVisible) {
            int btnSize = 16;
            int btnX = cx + cw - btnSize - 4;
            int btnY = cy + 4;
            // Semi-transparent background effect (dark rectangle)
            gfx->fillRect(btnX, btnY, btnSize, btnSize, 0x4208);
            gfx->drawRect(btnX, btnY, btnSize, btnSize, 0xFFFF);
            // X symbol
            gfx->drawLine(btnX + 4, btnY + 4, btnX + btnSize - 5, btnY + btnSize - 5, 0xFFFF);
            gfx->drawLine(btnX + btnSize - 5, btnY + 4, btnX + 4, btnY + btnSize - 5, 0xFFFF);
          }
        } else {
          gfx->setTextColor(0xF800);
          gfx->setCursor(cx + 4, cy + ch/2 - 4);
          gfx->print("Load failed");
        }
      }
      break;
    case 11:  // Snake game
      {
        // Calculate cell size to fit available space with info panel
        int infoW = 40;
        int availW = cw - infoW - 6;
        int availH = ch - 4;
        int cellSize = min(availW / SNAKE_COLS, availH / SNAKE_ROWS);
        if (cellSize < 2) cellSize = 2;
        int boardW = SNAKE_COLS * cellSize;
        int boardH = SNAKE_ROWS * cellSize;
        int boardX = cx + 2;
        int boardY = cy + (ch - boardH) / 2;
        int infoX = boardX + boardW + 4;
        
        // Require fullscreen mode
        if (!w.fullscreen) {
          gfx->setTextColor(COLOR_ACCENT);
          gfx->setCursor(cx + 4, cy + ch/2 - 20);
          gfx->print("Please be in");
          gfx->setCursor(cx + 4, cy + ch/2 - 8);
          gfx->print("fullscreen to");
          gfx->setCursor(cx + 4, cy + ch/2 + 4);
          gfx->print("use this app.");
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 4, cy + ch/2 + 20);
          gfx->print("Press SET");
        } else if (snakeShowMenu && !snakePlaying) {
          // Main menu
          gfx->setTextColor(COLOR_SNAKE);
          gfx->setCursor(cx + cw/2 - 18, cy + 20);
          gfx->print("SNAKE");
          
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + cw/2 - 30, cy + 45);
          gfx->print("High: ");
          gfx->print(snakeHighScore);
          
          int btnW = 60, btnH = 18;
          int btnX = cx + cw/2 - btnW/2;
          int btnY = cy + 65;
          gfx->fillRect(btnX, btnY, btnW, btnH, 0x07E0);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 12, btnY + 5);
          gfx->print("START");
          
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + 10, cy + 95);
          gfx->print("Arrows = Move");
          gfx->setCursor(cx + 10, cy + 107);
          gfx->print("MID = Pause");
        } else if (snakeGameOver) {
          gfx->setTextColor(0xF800);
          gfx->setCursor(cx + cw/2 - 30, cy + 20);
          gfx->print("GAME OVER");
          
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(cx + cw/2 - 30, cy + 45);
          gfx->print("Score: ");
          gfx->print(snakeScore);
          
          int btnW = 60, btnH = 18;
          int btnX = cx + cw/2 - btnW/2;
          int btnY = cy + 70;
          uint16_t btnColor = (snakeMenuSel == 0) ? 0x07E0 : 0x4208;
          gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 12, btnY + 5);
          gfx->print("RETRY");
          
          btnY = cy + 95;
          btnColor = (snakeMenuSel == 1) ? 0x07E0 : 0x4208;
          gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
          gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
          gfx->setTextColor(0xFFFF);
          gfx->setCursor(btnX + 15, btnY + 5);
          gfx->print("MENU");
        } else if (snakePlaying) {
          // Draw board border
          gfx->drawRect(boardX - 1, boardY - 1, boardW + 2, boardH + 2, COLOR_HIGHLIGHT);
          
          // Draw food
          gfx->fillRect(boardX + snakeFoodX * cellSize, boardY + snakeFoodY * cellSize, cellSize, cellSize, 0xF800);
          
          // Draw snake
          for (int i = 0; i < snakeLen; i++) {
            uint16_t c = (i == 0) ? 0x07E0 : 0x03E0;
            gfx->fillRect(boardX + snakeX[i] * cellSize, boardY + snakeY[i] * cellSize, cellSize, cellSize, c);
          }
          
          // Draw score
          gfx->setTextColor(COLOR_TEXT);
          gfx->setCursor(infoX, boardY);
          gfx->print("Score");
          gfx->setCursor(infoX, boardY + 12);
          gfx->print(snakeScore);
          
          // Pause overlay
          if (snakePaused) {
            gfx->fillRect(cx + cw/2 - 40, cy + ch/2 - 30, 80, 60, 0x0000);
            gfx->drawRect(cx + cw/2 - 40, cy + ch/2 - 30, 80, 60, COLOR_ACCENT);
            gfx->setTextColor(COLOR_ACCENT);
            gfx->setCursor(cx + cw/2 - 20, cy + ch/2 - 20);
            gfx->print("PAUSED");
            
            int btnW = 70, btnH = 18;
            int btnX = cx + cw/2 - btnW/2;
            int btnY = cy + ch/2 - 5;
            uint16_t btnColor = (snakeMenuSel == 0) ? 0x07E0 : 0x4208;
            gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
            gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
            gfx->setTextColor(0xFFFF);
            gfx->setCursor(btnX + 14, btnY + 5);
            gfx->print("RESUME");
            
            btnY = cy + ch/2 + 18;
            btnColor = (snakeMenuSel == 1) ? 0x07E0 : 0x4208;
            gfx->fillRect(btnX, btnY, btnW, btnH, btnColor);
            gfx->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
            gfx->setTextColor(0xFFFF);
            gfx->setCursor(btnX + 20, btnY + 5);
            gfx->print("MENU");
          }
        }
      }
      break;
    case 5:  // Music - Win98 style track list
      {
        int mItemH = w.maximized ? 14 : 10;
        int mBtnH = w.maximized ? 14 : 10;
        int scrollW = 8;
        int listH = ch - mBtnH - 4;
        int maxVisible = listH / mItemH;
        // Track list - Win98 style selection
        for (int i = 0; i < maxVisible && (i + musicScrollOffset) < NUM_TRACKS; i++) {
          int idx = i + musicScrollOffset;
          int iy = cy + 2 + i * mItemH;
          if (iy + mItemH > cy + ch - mBtnH - 2) break;
          bool sel = (idx == musicTrack);
          if (sel) gfx->fillRect(cx + 2, iy, cw - scrollW - 4, mItemH - 1, DESK_TITLEBAR_COLOR);
          gfx->setTextColor(sel ? 0xFFFF : 0x0000);
          gfx->setCursor(cx + 4, iy + 1);
          int maxTrackChars = w.maximized ? 18 : 9;
          for (int j = 0; j < maxTrackChars && trackNames[idx][j] != '\0'; j++) {
            gfx->print(trackNames[idx][j]);
          }
        }
        // Vertical scrollbar - Win98 style
        if (NUM_TRACKS > maxVisible) {
          int sbX = cx + cw - scrollW - 1;
          int sbY = cy + 2;
          int sbH = listH;
          gfx->fillRect(sbX, sbY, scrollW, sbH, DESK_BTN_FACE);
          gfx->drawFastHLine(sbX, sbY, scrollW, DESK_BTN_DARK);
          gfx->drawFastVLine(sbX, sbY, sbH, DESK_BTN_DARK);
          int thumbH = max(10, sbH * maxVisible / NUM_TRACKS);
          int thumbY = sbY + (sbH - thumbH) * musicScrollOffset / max(1, NUM_TRACKS - maxVisible);
          // 3D thumb
          gfx->fillRect(sbX + 1, thumbY, scrollW - 2, thumbH, DESK_BTN_FACE);
          gfx->drawFastHLine(sbX + 1, thumbY, scrollW - 2, DESK_BTN_LIGHT);
          gfx->drawFastVLine(sbX + 1, thumbY, thumbH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(sbX + 1, thumbY + thumbH - 1, scrollW - 2, DESK_BTN_DARK);
          gfx->drawFastVLine(sbX + scrollW - 2, thumbY, thumbH, DESK_BTN_DARK);
        }
        // Play button - Win98 3D style
        int mPlayBtnH = w.maximized ? 14 : 10;
        int btnY = cy + ch - mPlayBtnH - 2;
        gfx->fillRect(cx + 2, btnY, cw - 4, mPlayBtnH, 0xAFE5);
        gfx->drawFastHLine(cx + 2, btnY, cw - 4, DESK_BTN_LIGHT);
        gfx->drawFastVLine(cx + 2, btnY, mPlayBtnH, DESK_BTN_LIGHT);
        gfx->drawFastHLine(cx + 2, btnY + mPlayBtnH - 1, cw - 4, 0x0400);
        gfx->drawFastVLine(cx + cw - 3, btnY, mPlayBtnH, 0x0400);
        gfx->setTextColor(0x0000);
        gfx->setCursor(cx + cw/2 - charW * 2, btnY + 2);
        gfx->print("PLAY");
      }
      break;
    case 6:  // Files - Win98 style file explorer
      {
        // Auto-refresh is handled in main desktop loop (drawDesktopApp)
        
        if (!sdCardMounted) {
          gfx->setTextSize(2);
          gfx->setTextColor(0xF800);
          gfx->setCursor(cx + 4, cy + ch/2 - 8);
          gfx->print("No SD");
          gfx->setTextSize(1);
        } else {
          int fItemH = w.maximized ? 14 : 12;
          int pathH = w.maximized ? 14 : 12;
          int btnH = w.maximized ? 14 : 12;
          int maxVisible = (ch - pathH - btnH - 4) / fItemH;
          // Path bar - sunken 3D style
          gfx->fillRect(cx + 1, cy + 1, cw - 2, pathH, 0xFFFF);
          gfx->drawFastHLine(cx + 1, cy + 1, cw - 2, DESK_BTN_DARK);
          gfx->drawFastVLine(cx + 1, cy + 1, pathH, DESK_BTN_DARK);
          gfx->setCursor(cx + 3, cy + 3);
          gfx->setTextColor(0x0000);
          int pathLen = strlen(currentPath);
          int maxPathChars = (cw - 6) / charW;
          if (pathLen > maxPathChars) {
            gfx->print("...");
            for (int i = pathLen - maxPathChars + 3; i < pathLen; i++) gfx->print(currentPath[i]);
          } else {
            gfx->print(currentPath);
          }
          // File list - Win98 style selection
          int scrollW = 8;
          int listY = cy + pathH + 2;
          int listH = ch - pathH - btnH - 4;
          for (int i = 0; i < maxVisible && (i + fileScrollOffset) < fileCount; i++) {
            int idx = i + fileScrollOffset;
            int iy = listY + i * fItemH;
            bool sel = (idx == fileSelected);
            if (sel) gfx->fillRect(cx + 2, iy, cw - scrollW - 4, fItemH - 1, DESK_TITLEBAR_COLOR);
            // Files: black text, Directories: brown/orange, Selected: white
            gfx->setTextColor(sel ? 0xFFFF : (fileIsDir[idx] ? 0xC400 : COLOR_TEXT));
            gfx->setCursor(cx + 4, iy + 2);
            gfx->print(fileIsDir[idx] ? "> " : "  ");
            int maxNameChars = (cw - scrollW - 20) / charW;
            for (int j = 0; j < maxNameChars && fileNames[idx][j] != '\0'; j++) {
              gfx->print(fileNames[idx][j]);
            }
          }
          // Vertical scrollbar - Win98 style
          if (fileCount > maxVisible) {
            int sbX = cx + cw - scrollW - 1;
            int sbY = listY;
            int sbH = listH;
            gfx->fillRect(sbX, sbY, scrollW, sbH, DESK_BTN_FACE);
            gfx->drawFastHLine(sbX, sbY, scrollW, DESK_BTN_DARK);
            gfx->drawFastVLine(sbX, sbY, sbH, DESK_BTN_DARK);
            int thumbH = max(10, sbH * maxVisible / fileCount);
            int thumbY = sbY + (sbH - thumbH) * fileScrollOffset / max(1, fileCount - maxVisible);
            // 3D thumb
            gfx->fillRect(sbX + 1, thumbY, scrollW - 2, thumbH, DESK_BTN_FACE);
            gfx->drawFastHLine(sbX + 1, thumbY, scrollW - 2, DESK_BTN_LIGHT);
            gfx->drawFastVLine(sbX + 1, thumbY, thumbH, DESK_BTN_LIGHT);
            gfx->drawFastHLine(sbX + 1, thumbY + thumbH - 1, scrollW - 2, DESK_BTN_DARK);
            gfx->drawFastVLine(sbX + scrollW - 2, thumbY, thumbH, DESK_BTN_DARK);
          }
          // Bottom buttons - Win98 3D style
          int btnY = cy + ch - btnH - 1;
          int btnW = (cw - 12) / 5;
          // HOME (/) button - 3D cyan
          gfx->fillRect(cx + 2, btnY, btnW, btnH, 0x07FF);
          gfx->drawFastHLine(cx + 2, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(cx + 2, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(cx + 2, btnY + btnH - 1, btnW, 0x0410);
          gfx->drawFastVLine(cx + 1 + btnW, btnY, btnH, 0x0410);
          gfx->setTextColor(0x0000);
          gfx->setCursor(cx + 2 + btnW/2 - 3, btnY + 2);
          gfx->print("/");
          // UP button - 3D gray
          gfx->fillRect(cx + 4 + btnW, btnY, btnW, btnH, DESK_BTN_FACE);
          gfx->drawFastHLine(cx + 4 + btnW, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(cx + 4 + btnW, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(cx + 4 + btnW, btnY + btnH - 1, btnW, DESK_BTN_DARK);
          gfx->drawFastVLine(cx + 3 + btnW * 2, btnY, btnH, DESK_BTN_DARK);
          gfx->setTextColor(0x0000);
          gfx->setCursor(cx + 4 + btnW + btnW/2 - 6, btnY + 2);
          gfx->print("UP");
          // RENAME button - 3D yellow
          gfx->fillRect(cx + 6 + btnW * 2, btnY, btnW, btnH, 0xFFE0);
          gfx->drawFastHLine(cx + 6 + btnW * 2, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(cx + 6 + btnW * 2, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(cx + 6 + btnW * 2, btnY + btnH - 1, btnW, 0xC400);
          gfx->drawFastVLine(cx + 5 + btnW * 3, btnY, btnH, 0xC400);
          gfx->setTextColor(0x0000);
          gfx->setCursor(cx + 6 + btnW * 2 + btnW/2 - 6, btnY + 2);
          gfx->print("RN");
          // NEW button - 3D green
          gfx->fillRect(cx + 8 + btnW * 3, btnY, btnW, btnH, 0xAFE5);
          gfx->drawFastHLine(cx + 8 + btnW * 3, btnY, btnW, DESK_BTN_LIGHT);
          gfx->drawFastVLine(cx + 8 + btnW * 3, btnY, btnH, DESK_BTN_LIGHT);
          gfx->drawFastHLine(cx + 8 + btnW * 3, btnY + btnH - 1, btnW, 0x0400);
          gfx->drawFastVLine(cx + 7 + btnW * 4, btnY, btnH, 0x0400);
          gfx->setTextColor(0x0000);  // Black text on green button
          gfx->setCursor(cx + 8 + btnW * 3 + btnW/2 - 6, btnY + 2);
          gfx->print("NW");
          // DEL button - 3D red
          gfx->fillRect(cx + 10 + btnW * 4, btnY, btnW, btnH, 0xFBE0);
          gfx->drawFastHLine(cx + 10 + btnW * 4, btnY, btnW, 0xFFFF);
          gfx->drawFastVLine(cx + 10 + btnW * 4, btnY, btnH, 0xFFFF);
          gfx->drawFastHLine(cx + 10 + btnW * 4, btnY + btnH - 1, btnW, 0xA000);
          gfx->drawFastVLine(cx + 9 + btnW * 5, btnY, btnH, 0xA000);
          gfx->setTextColor(0x8000);
          gfx->setCursor(cx + 10 + btnW * 4 + btnW/2 - 6, btnY + 2);
          gfx->print("DL");
          
          // Delete confirmation overlay
          if (deleteConfirmMode && fileSelected < fileCount) {
            int popW = cw - 16;
            int popH = 36;
            int popX = cx + 8;
            int popY = cy + ch/2 - popH/2;
            gfx->fillRect(popX, popY, popW, popH, DESK_WINDOW_COLOR);
            gfx->drawRect(popX, popY, popW, popH, 0xF800);
            gfx->setTextColor(0xF800);
            gfx->setCursor(popX + 4, popY + 3);
            gfx->print("Delete?");
            // Show filename (truncated)
            gfx->setTextColor(COLOR_TEXT);
            gfx->setCursor(popX + 4, popY + 13);
            int maxC = (popW - 8) / 6;
            for (int i = 0; i < maxC && fileNames[fileSelected][i]; i++) {
              gfx->print(fileNames[fileSelected][i]);
            }
            // YES button
            int btnW2 = (popW - 12) / 2;
            gfx->fillRect(popX + 4, popY + 24, btnW2, 10, 0x07E0);
            gfx->drawRect(popX + 4, popY + 24, btnW2, 10, 0xFFFF);
            gfx->setTextColor(0x0000);
            gfx->setCursor(popX + 4 + btnW2/2 - 9, popY + 25);
            gfx->print("YES");
            // NO button
            gfx->fillRect(popX + 8 + btnW2, popY + 24, btnW2, 10, 0xF800);
            gfx->drawRect(popX + 8 + btnW2, popY + 24, btnW2, 10, 0xFFFF);
            gfx->setTextColor(0xFFFF);
            gfx->setCursor(popX + 8 + btnW2 + btnW2/2 - 6, popY + 25);
            gfx->print("NO");
          }
        }
      }
      break;
  }
  
  // Draw resize grip for this window (if not maximized)
  // Drawing it here ensures windows in front will cover it
  if (!w.maximized) {
    bool hoverResize = deskPtrHitsResize(winIdx);
    if (hoverResize) {
      // Expanded grip when hovering - larger and brighter
      gfx->fillRect(x + ww - 2, y + wh - 2, 8, 8, 0xC618);
      gfx->drawRect(x + ww - 2, y + wh - 2, 8, 8, 0xFFFF);
      gfx->drawLine(x + ww, y + wh + 4, x + ww + 4, y + wh, 0x0000);
      gfx->drawLine(x + ww - 1, y + wh + 4, x + ww + 4, y + wh - 1, 0x0000);
    } else {
      // Normal small grip
      for (int j = 0; j < 3; j++) {
        gfx->drawPixel(x + ww + j, y + wh + 2, 0x8410);
        gfx->drawPixel(x + ww + 2, y + wh + j, 0x8410);
      }
      gfx->drawLine(x + ww - 1, y + wh, x + ww + 2, y + wh, 0x8410);
      gfx->drawLine(x + ww, y + wh - 1, x + ww, y + wh + 2, 0x8410);
    }
  }
}

// Check if any window is covering the taskbar area
bool isTaskbarCovered() {
  for (int8_t i = 0; i < deskWinCount; i++) {
    int8_t idx = deskWinOrder[i];
    if (deskWins[idx].open) {
      DeskWindow& w = deskWins[idx];
      if (w.fullscreen || w.maximized) return true;
      // Check if window extends into taskbar area
      if (w.y + w.h > SCREEN_HEIGHT - DESK_TASKBAR_H) return true;
    }
  }
  return false;
}

// Update taskbar hidden state (call before drawing)
void updateTaskbarState() {
  bool shouldHide = isTaskbarCovered();
  
  // Trigger area: only the very bottom edge (2 pixels) when taskbar is hidden
  // But if taskbar is visible, check full taskbar area
  bool cursorAtBottomEdge = (mouseY >= SCREEN_HEIGHT - 2);
  bool cursorInTaskbar = taskbarHidden ? cursorAtBottomEdge : (mouseY >= SCREEN_HEIGHT - DESK_TASKBAR_H);
  bool cursorInStartMenu = false;
  
  // Check if cursor is in start menu area
  if (startMenuOpen) {
    int colW = 52;
    int itemH = 14;
    int rows = (START_MENU_ITEMS + 1) / 2;
    int menuW = colW * 2 + 6;
    int menuH = rows * itemH + 6;
    int menuY = SCREEN_HEIGHT - DESK_TASKBAR_H - menuH;
    if (mouseX >= 2 && mouseX < 2 + menuW && mouseY >= menuY) {
      cursorInStartMenu = true;
    }
  }
  
  // Show taskbar if cursor is in taskbar area or start menu area
  if (cursorInTaskbar || cursorInStartMenu) {
    shouldHide = false;
  }
  
  // Close start menu if cursor leaves it AND leaves taskbar area
  if (startMenuOpen && !cursorInTaskbar && !cursorInStartMenu) {
    startMenuOpen = false;
  }
  
  taskbarHidden = shouldHide;
}

// Draw 3D beveled button (Win98 style) - uses global gfx pointer
void draw3DButton(int x, int y, int w, int h, bool pressed) {
  gfx->fillRect(x, y, w, h, DESK_BTN_FACE);
  if (pressed) {
    // Pressed: dark top/left, light bottom/right (inverted)
    gfx->drawFastHLine(x, y, w, DESK_BTN_DARK);
    gfx->drawFastVLine(x, y, h, DESK_BTN_DARK);
    gfx->drawFastHLine(x, y + h - 1, w, DESK_BTN_LIGHT);
    gfx->drawFastVLine(x + w - 1, y, h, DESK_BTN_LIGHT);
  } else {
    // Normal: light top/left, dark bottom/right
    gfx->drawFastHLine(x, y, w, DESK_BTN_LIGHT);
    gfx->drawFastVLine(x, y, h, DESK_BTN_LIGHT);
    gfx->drawFastHLine(x, y + h - 1, w, DESK_BTN_DARK);
    gfx->drawFastVLine(x + w - 1, y, h, DESK_BTN_DARK);
  }
}

// Draw taskbar
void deskDrawTaskbar() {
  if (taskbarHidden) return;  // Don't draw if hidden
  
  int ty = SCREEN_HEIGHT - DESK_TASKBAR_H;
  // Taskbar background with 3D top edge
  gfx->fillRect(0, ty, SCREEN_WIDTH, DESK_TASKBAR_H, DESK_TASKBAR_COLOR);
  gfx->drawFastHLine(0, ty, SCREEN_WIDTH, DESK_BTN_LIGHT);  // Top highlight
  gfx->drawFastHLine(0, ty + 1, SCREEN_WIDTH, DESK_BTN_DARK);  // Inner shadow line
  
  // Start button with 3D effect
  draw3DButton(2, ty + 2, DESK_START_W, DESK_TASKBAR_H - 4, startMenuOpen);
  gfx->setTextSize(1);
  gfx->setTextColor(0x0000);
  gfx->setCursor(startMenuOpen ? 8 : 6, ty + 6);
  gfx->print("Start");
  
  // Window buttons in taskbar
  int bx = DESK_START_W + 6;
  for (int8_t i = 0; i < deskWinCount && bx < SCREEN_WIDTH - 40; i++) {
    int8_t idx = deskWinOrder[i];
    if (deskWins[idx].open) {
      bool active = (idx == activeWin);
      draw3DButton(bx, ty + 2, 40, DESK_TASKBAR_H - 4, active);
      gfx->setTextColor(0x0000);
      gfx->setCursor(bx + 3, ty + 6);
      // Truncate title
      char shortTitle[6];
      strncpy(shortTitle, deskWins[idx].title, 5);
      shortTitle[5] = '\0';
      gfx->print(shortTitle);
      bx += 42;
    }
  }
  
  // Clock area with sunken 3D effect
  int clockX = SCREEN_WIDTH - 38;
  int clockW = 36;
  gfx->fillRect(clockX, ty + 2, clockW, DESK_TASKBAR_H - 4, DESK_TASKBAR_COLOR);
  gfx->drawFastHLine(clockX, ty + 2, clockW, DESK_BTN_DARK);
  gfx->drawFastVLine(clockX, ty + 2, DESK_TASKBAR_H - 4, DESK_BTN_DARK);
  gfx->drawFastHLine(clockX, ty + DESK_TASKBAR_H - 3, clockW, DESK_BTN_LIGHT);
  gfx->drawFastVLine(clockX + clockW - 1, ty + 2, DESK_TASKBAR_H - 4, DESK_BTN_LIGHT);
  
  // Time and date
  gfx->setTextColor(0x0000);
  char timeStr[6];
  snprintf(timeStr, 6, "%d:%02d", hours, minutes);
  gfx->setCursor(clockX + 4, ty + 4);
  gfx->print(timeStr);
  char dateStr[6];
  snprintf(dateStr, 6, "%d/%d", month, day);
  gfx->setCursor(clockX + 4, ty + 12);
  gfx->print(dateStr);
}

// Draw start menu (two columns) - Win98 style
void deskDrawStartMenu() {
  if (!startMenuOpen || taskbarHidden) return;  // Don't show if taskbar is hidden
  
  int colW = 52;  // Width per column
  int itemH = 14;  // Height per item
  int rows = (START_MENU_ITEMS + 1) / 2;  // Rows needed
  int menuW = colW * 2 + 6;
  int menuH = rows * itemH + 6;
  int menuY = SCREEN_HEIGHT - DESK_TASKBAR_H - menuH;
  
  // Menu background with 3D raised border
  gfx->fillRect(2, menuY, menuW, menuH, DESK_TASKBAR_COLOR);
  // Outer 3D border
  gfx->drawFastHLine(2, menuY, menuW, DESK_BTN_LIGHT);
  gfx->drawFastVLine(2, menuY, menuH, DESK_BTN_LIGHT);
  gfx->drawFastHLine(2, menuY + menuH - 1, menuW, DESK_BTN_DARK);
  gfx->drawFastVLine(2 + menuW - 1, menuY, menuH, DESK_BTN_DARK);
  // Inner border
  gfx->drawFastHLine(3, menuY + 1, menuW - 2, 0xFFFF);
  gfx->drawFastVLine(3, menuY + 1, menuH - 2, 0xFFFF);
  
  // Menu items in two columns
  gfx->setTextSize(1);
  for (int i = 0; i < START_MENU_ITEMS; i++) {
    int col = i / rows;  // 0 for left column, 1 for right
    int row = i % rows;
    int ix = 4 + col * colW;
    int iy = menuY + 3 + row * itemH;
    bool sel = (i == startMenuSel);
    if (sel) {
      gfx->fillRect(ix - 1, iy, colW - 1, itemH - 1, DESK_TITLEBAR_COLOR);
      gfx->setTextColor(0xFFFF);
    } else {
      gfx->setTextColor(0x0000);
    }
    gfx->setCursor(ix + 2, iy + 3);
    gfx->print(startMenuItems[i]);
  }
}

// Draw "Back" icon on desktop - REMOVED (no exit shortcut)
void deskDrawBackIcon() {
  // No longer drawing back icon - exit only via Start menu
}

// Full desktop redraw with double buffering
void deskFullRedraw() {
  // Safety check - ensure buffer and gfx are valid
  if (!bufferReady) {
    initDoubleBuffer();
  }
  if (!gfx) {
    gfx = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  }
  
  // Update taskbar state first so we know if it will be hidden
  updateTaskbarState();
  
  // Draw everything to buffer first
  // Desktop background - extend to full screen if taskbar is hidden
  int bgHeight = taskbarHidden ? SCREEN_HEIGHT : (SCREEN_HEIGHT - DESK_TASKBAR_H);
  gfx->fillRect(0, 0, SCREEN_WIDTH, bgHeight, DESK_DESKTOP_COLOR);
  
  // ArduinOS logo (bottom-right corner of desktop, above taskbar)
  // The bitmap has 1=background, 0=text, so we draw inverted (0 bits = white)
  int logoX = SCREEN_WIDTH - LOGO_WIDTH - 4;
  int logoY = SCREEN_HEIGHT - DESK_TASKBAR_H - LOGO_HEIGHT - 2;
  for (int y = 0; y < LOGO_HEIGHT; y++) {
    for (int x = 0; x < LOGO_WIDTH; x++) {
      int byteIndex = y * ((LOGO_WIDTH + 7) / 8) + (x / 8);
      int bitIndex = 7 - (x % 8);
      uint8_t b = pgm_read_byte(&epd_bitmap_logoarduin[byteIndex]);
      if (!((b >> bitIndex) & 1)) {  // Inverted: 0 = draw white
        gfx->drawPixel(logoX + x, logoY + y, 0xFFFF);  // White logo
      }
    }
    if (y % 20 == 0) yield();  // Prevent watchdog timeout
  }
  
  // Desktop shortcut icon for Files (top-left corner) - Hard drive icon
  if (sdCardMounted) {
    int iconX = 4;
    int iconY = 4;
    // Hard drive body (gray box)
    gfx->fillRect(iconX, iconY, 28, 20, 0x8410);  // Gray body
    gfx->drawRect(iconX, iconY, 28, 20, 0x0000);  // Black outline
    // Drive slot/vent lines
    gfx->drawFastHLine(iconX + 3, iconY + 4, 16, 0x4208);
    gfx->drawFastHLine(iconX + 3, iconY + 7, 16, 0x4208);
    gfx->drawFastHLine(iconX + 3, iconY + 10, 16, 0x4208);
    // Activity LED (green)
    gfx->fillRect(iconX + 22, iconY + 14, 4, 4, 0x07E0);
    // Label below
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_TEXT);
    gfx->setCursor(iconX + 2, iconY + 22);
    gfx->print("Disk");
  }
  
  // Windows (back to front)
  for (int8_t i = deskWinCount - 1; i >= 0; i--) {
    deskDrawWindow(deskWinOrder[i]);
  }
  
  // Taskbar
  deskDrawTaskbar();
  
  // Start menu
  deskDrawStartMenu();
  
  // Notification popup (drawn on top of everything except mouse) - Win98 style
  if (notificationPopup) {
    int popW = 120;
    int popH = 50;
    int popX = (SCREEN_WIDTH - popW) / 2;
    int popY = (SCREEN_HEIGHT - DESK_TASKBAR_H - popH) / 2;
    // Popup background with 3D raised border
    gfx->fillRect(popX, popY, popW, popH, DESK_BTN_FACE);
    gfx->drawFastHLine(popX, popY, popW, DESK_BTN_LIGHT);
    gfx->drawFastVLine(popX, popY, popH, DESK_BTN_LIGHT);
    gfx->drawFastHLine(popX, popY + popH - 1, popW, DESK_BTN_DARK);
    gfx->drawFastVLine(popX + popW - 1, popY, popH, DESK_BTN_DARK);
    gfx->drawRect(popX, popY, popW, popH, 0x0000);
    // Title bar
    gfx->fillRect(popX + 2, popY + 2, popW - 4, 12, DESK_TITLEBAR_COLOR);
    gfx->setTextSize(1);
    gfx->setTextColor(0xFFFF);
    gfx->setCursor(popX + 4, popY + 4);
    gfx->print("Message");
    // Message text
    gfx->setTextColor(COLOR_TEXT);
    int msgLen = strlen(notificationMsg);
    int msgX = popX + (popW - msgLen * 6) / 2;
    gfx->setCursor(msgX, popY + 18);
    gfx->print(notificationMsg);
    // OK button with 3D effect
    int btnW = 40;
    int btnX = popX + (popW - btnW) / 2;
    int btnY = popY + 32;
    gfx->fillRect(btnX, btnY, btnW, 14, DESK_BTN_FACE);
    gfx->drawFastHLine(btnX, btnY, btnW, DESK_BTN_LIGHT);
    gfx->drawFastVLine(btnX, btnY, 14, DESK_BTN_LIGHT);
    gfx->drawFastHLine(btnX, btnY + 13, btnW, DESK_BTN_DARK);
    gfx->drawFastVLine(btnX + btnW - 1, btnY, 14, DESK_BTN_DARK);
    gfx->setTextColor(0x0000);
    gfx->setCursor(btnX + 14, btnY + 3);
    gfx->print("OK");
  }
  
  // Mouse pointer (hidden when cursorHidden is true, for games)
  if (!cursorHidden) {
    drawMousePointer(mouseX, mouseY, false);
  }
  
  // Copy buffer to screen in one operation (eliminates flicker)
  flushBuffer();
}

// Main desktop app function
void drawDesktopApp() {
  static bool needFullDraw = true;
  static unsigned long lastFrame = 0;
  static bool initialized = false;
  
  // First entry - initialize desktop mode
  if (!initialized || !desktopMode) {
    initialized = true;
    desktopMode = true;
    needFullDraw = true;
    mouseX = 120; mouseY = 67;
    prevMouseX = mouseX; prevMouseY = mouseY;
    startMenuOpen = false;
    startMenuSel = 0;
    dragging = false;
    resizing = false;
    mouseDown = false;
    mouseHoldStart = 0;
    // Allocate buffer lazily when entering desktop mode
    initDoubleBuffer();
    gfx = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  }
  
  // Check global redraw flag (set by async operations like WiFi connect)
  if (deskNeedRedraw) {
    needFullDraw = true;
    deskNeedRedraw = false;
  }
  
  // Delayed file refresh after screenshot (0.5s delay to avoid blocking)
  if (screenshotTakenTime > 0 && millis() - screenshotTakenTime >= 500) {
    screenshotTakenTime = 0;  // Reset so we only refresh once
    // Only refresh if Files window is open
    for (int i = 0; i < MAX_DESK_WINDOWS; i++) {
      if (deskWins[i].open && deskWins[i].appType == 6) {
        int oldSelected = fileSelected;
        int oldScroll = fileScrollOffset;
        scanDirectory(currentPath);
        if (oldSelected < fileCount) fileSelected = oldSelected;
        else if (fileCount > 0) fileSelected = fileCount - 1;
        if (oldScroll < fileCount) fileScrollOffset = oldScroll;
        needFullDraw = true;
        break;
      }
    }
  }
  
  if (needFullDraw) {
    deskFullRedraw();
    needFullDraw = false;
    lastFrame = millis();
    updateSound();  // Keep sound system updated
    return;
  }
  
  // Frame rate limit - adaptive based on activity
  // 30fps normally, 20fps when dragging/resizing (reduces CPU load)
  unsigned long now = millis();
  int frameDelay = (dragging || resizing) ? 50 : 33;  // 20fps vs 30fps
  if (now - lastFrame < (unsigned long)frameDelay) {
    updateSound();  // Keep sound system updated even when frame-limited
    yield();  // Prevent watchdog timeout during rapid input
    return;
  }
  lastFrame = now;
  
  // Watchdog feed - critical to prevent crash from accumulated input processing
  yield();
  esp_task_wdt_reset();
  
  // Deferred save: periodically save dirty data to avoid freeze on window close
  if (dataDirty && (now - lastSaveTime >= SAVE_INTERVAL)) {
    yield();  // Give system time before slow SD operation
    saveOSData();
    yield();  // Give system time after slow SD operation
    dataDirty = false;
    lastSaveTime = now;
  }
  
  // Use cached joystick state (read once per frame in readJoystick())
  bool up = joyUp;
  bool down = joyDown;
  bool left = joyLeft;
  bool right = joyRight;
  bool mid = joyMid;
  bool rst = joyRst;
  bool set = joySet;
  static bool lastRst = false;
  static bool lastSet = false;
  
  // RST button toggles cursor visibility (for games)
  if (rst && !lastRst) {
    cursorHidden = !cursorHidden;
    playTick();
    needFullDraw = true;
  }
  lastRst = rst;
  
  // SET button: for Image viewer (appType 12), close the window; otherwise toggle fullscreen
  if (set && !lastSet && activeWin >= 0 && deskWins[activeWin].open) {
    if (deskWins[activeWin].appType == 12) {
      // Image viewer: SET closes the window
      playBack();
      deskCloseWindow(activeWin);
    } else {
      // Other apps: toggle fullscreen
      deskWins[activeWin].fullscreen = !deskWins[activeWin].fullscreen;
      if (deskWins[activeWin].fullscreen) {
        deskWins[activeWin].maximized = false;  // Fullscreen overrides maximized
      }
      playTick();
    }
    needFullDraw = true;
  }
  lastSet = set;
  
  // Update Tetris game if active
  tetrisUpdate();
  tetrisMusicUpdate();
  
  // Update Snake music if active
  snakeMusicUpdate();
  
  // Check if Tetris window is active and handle direct input
  if (activeWin >= 0 && deskWins[activeWin].appType == 9 && deskWins[activeWin].fullscreen) {
    static bool lastTetrisMid = false;
    static bool lastTetrisUp = false, lastTetrisDown = false, lastTetrisLeft = false, lastTetrisRight = false;
    
    // Menu navigation with up/down
    if (tetrisShowMenu && !tetrisPlaying) {
      // Main menu only has one button, no navigation needed
      if (mid && !lastTetrisMid) {
        // Start game
        playSelect();
        tetrisShowMenu = false;
        tetrisPlaying = true;
        tetrisGameOver = false;
        memset(tetrisBoard, 0, sizeof(tetrisBoard));
        tetrisScore = 0;
        tetrisLevel = 1;
        tetrisLines = 0;
        tetrisDropInterval = 500;
        tetrisSoftDropping = false;
        tetrisLastDrop = millis();
        tetrisSpawnPiece();
      }
    } else if (tetrisGameOver) {
      // Game over menu - navigate with up/down
      if (up && !lastTetrisUp) {
        tetrisMenuSel = 0;  // Retry
        playTick();
      }
      if (down && !lastTetrisDown) {
        tetrisMenuSel = 1;  // Menu
        playTick();
      }
      if (mid && !lastTetrisMid) {
        if (tetrisMenuSel == 0) {
          // Retry
          playSelect();
          tetrisPlaying = true;
          tetrisGameOver = false;
          memset(tetrisBoard, 0, sizeof(tetrisBoard));
          tetrisScore = 0;
          tetrisLevel = 1;
          tetrisLines = 0;
          tetrisDropInterval = 500;
          tetrisSoftDropping = false;
          tetrisLastDrop = millis();
          tetrisSpawnPiece();
        } else {
          // Return to menu
          playClick();
          tetrisShowMenu = true;
          tetrisPlaying = false;
          tetrisGameOver = false;
          tetrisMenuSel = 0;
        }
      }
    } else if (tetrisPaused) {
      // Pause menu - navigate with up/down
      if (up && !lastTetrisUp) {
        tetrisMenuSel = 0;  // Resume
        playTick();
      }
      if (down && !lastTetrisDown) {
        tetrisMenuSel = 1;  // Menu
        playTick();
      }
      if (mid && !lastTetrisMid) {
        if (tetrisMenuSel == 0) {
          // Resume game
          playSelect();
          tetrisPaused = false;
          tetrisLastDrop = millis();  // Reset drop timer
        } else {
          // Return to menu
          playClick();
          tetrisShowMenu = true;
          tetrisPlaying = false;
          tetrisPaused = false;
          tetrisMenuSel = 0;
        }
      }
      tetrisSoftDropping = false;
    } else if (tetrisPlaying) {
      // Game controls - no sound on movement to avoid interrupting music
      if (left && !lastTetrisLeft) { tetrisMove(-1); }
      if (right && !lastTetrisRight) { tetrisMove(1); }
      if (up && !lastTetrisUp) { tetrisRotate(); }
      // Down = soft drop (hold to fall faster)
      tetrisSoftDropping = down;
      
      // MID = open pause menu
      if (mid && !lastTetrisMid) {
        playClick();
        tetrisPaused = true;
        tetrisMenuSel = 0;  // Default to Resume
      }
    }
    
    if (!tetrisPlaying || tetrisPaused) {
      tetrisSoftDropping = false;
    }
    
    lastTetrisMid = mid;
    lastTetrisUp = up; lastTetrisDown = down; lastTetrisLeft = left; lastTetrisRight = right;
    needFullDraw = true;  // Tetris needs constant redraw
  }
  
  // Update Snake game if active
  snakeUpdate();
  
  // Check if Snake window is active and handle direct input
  if (activeWin >= 0 && deskWins[activeWin].appType == 11 && deskWins[activeWin].fullscreen) {
    static bool lastSnakeMid = false;
    static bool lastSnakeUp = false, lastSnakeDown = false, lastSnakeLeft = false, lastSnakeRight = false;
    
    if (snakeShowMenu && !snakePlaying) {
      if (mid && !lastSnakeMid) {
        playSelect();
        snakeInitGame();
        snakeShowMenu = false;
        snakePlaying = true;
      }
    } else if (snakeGameOver) {
      if (up && !lastSnakeUp) { snakeMenuSel = 0; playTick(); }
      if (down && !lastSnakeDown) { snakeMenuSel = 1; playTick(); }
      if (mid && !lastSnakeMid) {
        if (snakeMenuSel == 0) {
          playSelect();
          snakeInitGame();
          snakePlaying = true;
          snakeShowMenu = false;
        } else {
          playClick();
          snakeShowMenu = true;
          snakeGameOver = false;
          snakeMenuSel = 0;
        }
      }
    } else if (snakePaused) {
      if (up && !lastSnakeUp) { snakeMenuSel = 0; playTick(); }
      if (down && !lastSnakeDown) { snakeMenuSel = 1; playTick(); }
      if (mid && !lastSnakeMid) {
        if (snakeMenuSel == 0) {
          playSelect();
          snakePaused = false;
          snakeLastMove = millis();
        } else {
          playClick();
          snakeShowMenu = true;
          snakePlaying = false;
          snakePaused = false;
          snakeMenuSel = 0;
        }
      }
    } else if (snakePlaying) {
      if (up && !lastSnakeUp && snakeDir != 2) { snakeDir = 0; }
      if (down && !lastSnakeDown && snakeDir != 0) { snakeDir = 2; }
      if (left && !lastSnakeLeft && snakeDir != 1) { snakeDir = 3; }
      if (right && !lastSnakeRight && snakeDir != 3) { snakeDir = 1; }
      if (mid && !lastSnakeMid) {
        playClick();
        snakePaused = true;
        snakeMenuSel = 0;
      }
    }
    
    lastSnakeMid = mid;
    lastSnakeUp = up; lastSnakeDown = down; lastSnakeLeft = left; lastSnakeRight = right;
    needFullDraw = true;
  }
  
  // Paint mode - accelerated cursor movement and continuous drawing
  static unsigned long paintAccelStart = 0;
  static int paintLastDir = 0;
  
  if (activeWin >= 0 && deskWins[activeWin].appType == 10) {
    // Cursor movement with acceleration when holding direction (same as non-desktop Paint)
    int currentDir = 0;
    if (up) currentDir = 1;
    else if (down) currentDir = 2;
    else if (left) currentDir = 3;
    else if (right) currentDir = 4;
    
    if (currentDir > 0) {
      // Track acceleration - reset if direction changed
      if (currentDir != paintLastDir) {
        paintAccelStart = now;
        paintLastDir = currentDir;
      }
      
      // Calculate speed based on hold duration (accelerate from 2 to 12 pixels)
      unsigned long holdTime = now - paintAccelStart;
      int speed = 2;
      if (holdTime > 200) speed = 4;
      if (holdTime > 500) speed = 6;
      if (holdTime > 800) speed = 8;
      if (holdTime > 1200) speed = 10;
      if (holdTime > 1600) speed = 12;
      
      int dx = 0, dy = 0;
      if (currentDir == 3) dx = -speed;  // left
      if (currentDir == 4) dx = speed;   // right
      if (currentDir == 1) dy = -speed;  // up
      if (currentDir == 2) dy = speed;   // down
      
      mouseX = constrain(mouseX + dx, 0, SCREEN_WIDTH - 1);
      mouseY = constrain(mouseY + dy, 0, SCREEN_HEIGHT - 1);
      needFullDraw = true;
    } else {
      paintLastDir = 0;  // Reset when no direction held
    }
    
    // Continuous drawing while holding mid button
    if (mid) {
      DeskWindow& w = deskWins[activeWin];
      int16_t wx, wy, ww, wh;
      int cx, cy, cw, ch;
      if (w.fullscreen) {
        wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
        cx = 0; cy = 0; cw = SCREEN_WIDTH; ch = SCREEN_HEIGHT;
      } else if (w.maximized) {
        wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
        cx = wx + 2; cy = wy + DESK_TITLEBAR_H + 1;
        cw = ww - 4; ch = wh - DESK_TITLEBAR_H - 3;
      } else {
        wx = w.x; wy = w.y; ww = w.w; wh = w.h;
        cx = wx + 2; cy = wy + DESK_TITLEBAR_H + 1;
        cw = ww - 4; ch = wh - DESK_TITLEBAR_H - 3;
      }
      int lx = mouseX - cx, ly = mouseY - cy;
      
      int sidebarW = 20;
      int canvasAreaW = cw - sidebarW - 4;
      int canvasAreaH = ch - 4;
      int cellSize = min(canvasAreaW / PAINT_WIDTH, canvasAreaH / PAINT_HEIGHT);
      if (cellSize < 1) cellSize = 1;
      int canvasW = PAINT_WIDTH * cellSize;
      int canvasH = PAINT_HEIGHT * cellSize;
      int canvasX = 2;
      int canvasY = 2;
      
      // Draw on canvas while holding mid
      if (lx >= canvasX && lx < canvasX + canvasW &&
          ly >= canvasY && ly < canvasY + canvasH) {
        int px = (lx - canvasX) / cellSize;
        int py = (ly - canvasY) / cellSize;
        if (px >= 0 && px < PAINT_WIDTH && py >= 0 && py < PAINT_HEIGHT) {
          uint16_t newColor = paintEraser ? 0xFFFF : paintColor;
          if (paintCanvas[py][px] != newColor) {
            paintCanvas[py][px] = newColor;
            needFullDraw = true;
          }
        }
      }
    }
  }
  
  // Joystick only controls mouse cursor in desktop mode - apps use mouse clicks
  // Fixed mouse speed - slow down when over buttons/resize/keys for precision
  mouseSpeed = 4;
  bool hoveringResize = false;
  int8_t hitWinOrder = deskPtrHitsWindow();
  if (hitWinOrder >= 0) {
    int8_t winIdx = deskWinOrder[hitWinOrder];
    if (deskPtrHitsClose(winIdx) || deskPtrHitsMaximize(winIdx)) {
      mouseSpeed = 3;  // 25% slower over buttons
    }
    // Check if over Notes keyboard area (app type 4)
    DeskWindow& w = deskWins[winIdx];
    if (w.appType == 4) {
      bool largeKB = w.fullscreen || w.maximized;
      int16_t wx = w.fullscreen ? 0 : (w.maximized ? 0 : w.x);
      int16_t wy = w.fullscreen ? 0 : (w.maximized ? 0 : w.y);
      int cy = wy + (w.fullscreen ? 0 : DESK_TITLEBAR_H + 1);
      int noteDispH = largeKB ? 20 : 16;
      int keyboardY = cy + noteDispH + 2;
      if (mouseY >= keyboardY) {
        mouseSpeed = 2;  // 35% slower over keyboard keys
      }
    }
  }
  // Check resize corners on all windows (can be outside window bounds)
  for (int8_t i = 0; i < deskWinCount; i++) {
    int8_t idx = deskWinOrder[i];
    if (deskWins[idx].open && deskPtrHitsResize(idx)) {
      mouseSpeed = 3;  // 25% slower over resize corner
      hoveringResize = true;
      break;
    }
  }
  
  // Slow mouse over virtual keyboards (Notes app type 4, Editor app type 7, WiFi app type 8)
  for (int8_t i = deskWinCount - 1; i >= 0; i--) {
    int8_t idx = deskWinOrder[i];
    DeskWindow& w = deskWins[idx];
    if (w.open && (w.appType == 4 || w.appType == 7 || w.appType == 8)) {
      bool largeKB = w.fullscreen || w.maximized;
      int16_t wx = w.fullscreen ? 0 : (w.maximized ? 0 : w.x);
      int16_t wy = w.fullscreen ? 0 : (w.maximized ? 0 : w.y);
      int16_t ww = w.fullscreen ? SCREEN_WIDTH : (w.maximized ? SCREEN_WIDTH : w.w);
      int16_t wh = w.fullscreen ? SCREEN_HEIGHT : (w.maximized ? (SCREEN_HEIGHT - DESK_TASKBAR_H) : w.h);
      int16_t cy = wy + (w.fullscreen ? 0 : DESK_TITLEBAR_H + 1);
      int noteDispH = largeKB ? 20 : 16;
      int kbY = cy + noteDispH + 2;  // Keyboard starts after text display
      // Check if mouse is over keyboard area
      if (mouseX >= wx && mouseX < wx + ww && mouseY >= kbY && mouseY < wy + wh) {
        mouseSpeed = 3;  // 25% slower over keyboard
        break;
      }
    }
  }
  
  // Move mouse (only when cursor is visible)
  // Skip normal movement if Paint is active (Paint uses its own acceleration)
  bool paintActive = (activeWin >= 0 && deskWins[activeWin].appType == 10);
  prevMouseX = mouseX; prevMouseY = mouseY;
  if (!cursorHidden && !paintActive) {
    if (up) mouseY -= mouseSpeed;
    if (down) mouseY += mouseSpeed;
    if (left) mouseX -= mouseSpeed;
    if (right) mouseX += mouseSpeed;
  }
  
  // Clamp to screen
  if (mouseX < 0) mouseX = 0;
  if (mouseX >= SCREEN_WIDTH) mouseX = SCREEN_WIDTH - 1;
  if (mouseY < 0) mouseY = 0;
  if (mouseY >= SCREEN_HEIGHT) mouseY = SCREEN_HEIGHT - 1;
  
  // Handle dragging - only redraw if mouse actually moved
  if (dragging && dragWinIdx >= 0 && (mouseX != prevMouseX || mouseY != prevMouseY)) {
    DeskWindow& w = deskWins[dragWinIdx];
    w.x = mouseX - dragOffX;
    w.y = mouseY - dragOffY;
    // Keep on screen (allow dragging over taskbar)
    if (w.x < 0) w.x = 0;
    if (w.y < 0) w.y = 0;
    if (w.x + w.w > SCREEN_WIDTH) w.x = SCREEN_WIDTH - w.w;
    if (w.y + w.h > SCREEN_HEIGHT) w.y = SCREEN_HEIGHT - w.h;  // Allow over taskbar
    needFullDraw = true;
  }
  
  // Handle resizing - only redraw if mouse actually moved
  if (resizing && dragWinIdx >= 0 && (mouseX != prevMouseX || mouseY != prevMouseY)) {
    DeskWindow& w = deskWins[dragWinIdx];
    int16_t newW = mouseX - w.x + dragOffX;
    int16_t newH = mouseY - w.y + dragOffY;
    if (newW >= w.minW) w.w = newW;
    if (newH >= w.minH) w.h = newH;
    if (w.x + w.w > SCREEN_WIDTH) w.w = SCREEN_WIDTH - w.x;
    if (w.y + w.h > SCREEN_HEIGHT) w.h = SCREEN_HEIGHT - w.y;  // Allow over taskbar
    needFullDraw = true;
  }
  
  // Handle scrollbar dragging
  if (scrollDragging && (mouseX != prevMouseX || mouseY != prevMouseY)) {
    for (int i = 0; i < MAX_DESK_WINDOWS; i++) {
      if (deskWins[i].open && deskWins[i].appType == scrollDragApp) {
        DeskWindow& w = deskWins[i];
        int16_t wy = w.maximized ? 0 : w.y;
        int16_t wh = w.maximized ? (SCREEN_HEIGHT - DESK_TASKBAR_H) : w.h;
        int16_t cy = wy + DESK_TITLEBAR_H + 1;
        int16_t ch = wh - DESK_TITLEBAR_H - 3;
        
        if (scrollDragApp == 6) {  // File explorer
          int pathH = w.maximized ? 14 : 12;
          int btnH = w.maximized ? 14 : 12;
          int fItemH = w.maximized ? 14 : 12;
          int listY = cy + pathH + 2;
          int listH = ch - pathH - btnH - 4;
          int maxVisible = listH / fItemH;
          int relY = mouseY - listY;
          if (relY < 0) relY = 0;
          if (relY > listH) relY = listH;
          int maxScroll = max(0, fileCount - maxVisible);
          fileScrollOffset = relY * maxScroll / max(1, listH);
          if (fileScrollOffset > maxScroll) fileScrollOffset = maxScroll;
        } else if (scrollDragApp == 5) {  // Music
          int mItemH = w.maximized ? 14 : 10;
          int mBtnH = w.maximized ? 14 : 10;
          int listY = cy + 2;
          int listH = ch - mBtnH - 4;
          int maxVisible = listH / mItemH;
          int relY = mouseY - listY;
          if (relY < 0) relY = 0;
          if (relY > listH) relY = listH;
          int maxScroll = max(0, NUM_TRACKS - maxVisible);
          musicScrollOffset = relY * maxScroll / max(1, listH);
          if (musicScrollOffset > maxScroll) musicScrollOffset = maxScroll;
        } else if (scrollDragApp == 3) {  // TODO
          int itemH = w.maximized ? 14 : 10;
          int listY = cy + 2;
          int listH = ch - 4;
          int maxVisible = listH / itemH;
          int relY = mouseY - listY;
          if (relY < 0) relY = 0;
          if (relY > listH) relY = listH;
          int maxScroll = max(0, MAX_TODO - maxVisible);
          todoScrollOffset = relY * maxScroll / max(1, listH);
          if (todoScrollOffset > maxScroll) todoScrollOffset = maxScroll;
        } else if (scrollDragApp == 0) {  // Settings
          int itemH = w.maximized ? 14 : 10;
          int listY = cy + 2;
          int listH = ch - 4;
          int maxVisible = listH / itemH;
          int relY = mouseY - listY;
          if (relY < 0) relY = 0;
          if (relY > listH) relY = listH;
          int maxScroll = max(0, SETTINGS_ITEMS - maxVisible);
          settingsScrollOffset = relY * maxScroll / max(1, listH);
          if (settingsScrollOffset > maxScroll) settingsScrollOffset = maxScroll;
        }
        needFullDraw = true;
        break;
      }
    }
  }
  
  // Mouse click handling with hardware debounce
  static unsigned long lastDeskClick = 0;
  static unsigned long deskMidDebounceStart = 0;
  static bool deskMidDebounced = false;
  static bool deskMidWasLow = false;
  
  // Hardware debounce - require 50ms stable LOW
  if (mid && !deskMidWasLow) {
    deskMidDebounceStart = now;
    deskMidDebounced = false;
  }
  if (mid && !deskMidDebounced && now - deskMidDebounceStart >= 50) {
    deskMidDebounced = true;
  }
  if (!mid) {
    deskMidDebounced = false;
  }
  
  bool wasDown = mouseDown;
  mouseDown = deskMidDebounced;  // Only consider debounced state
  deskMidWasLow = mid;
  
  // Require 150ms between clicks to prevent double-click
  if (deskMidDebounced && !wasDown && now - lastDeskClick >= 150) {
    lastDeskClick = now;
    // Mouse down - start actions
    mouseDownTime = now;
    
    // Handle notification popup first (blocks other clicks)
    if (notificationPopup) {
      int popW = 100;
      int popH = 40;
      int popX = (SCREEN_WIDTH - popW) / 2;
      int popY = (SCREEN_HEIGHT - DESK_TASKBAR_H - popH) / 2;
      int btnW = 40;
      int btnX = popX + (popW - btnW) / 2;
      int btnY = popY + 24;
      // Check OK button
      if (mouseX >= btnX && mouseX < btnX + btnW && mouseY >= btnY && mouseY < btnY + 12) {
        playClick();
        notificationPopup = false;
        needFullDraw = true;
      }
      // Any click dismisses popup
      playClick();
      notificationPopup = false;
      needFullDraw = true;
      return;
    }
    
    // If currently scrollbar dragging, click anywhere stops it
    if (scrollDragging) {
      playClick();
      scrollDragging = false;
      scrollDragApp = -1;
      needFullDraw = true;
      return;
    }
    // If currently window dragging, click anywhere stops it
    if (dragging) {
      playClick();
      dragging = false;
      dragWinIdx = -1;
      needFullDraw = true;
      return;
    }
    // If currently resizing, click anywhere stops it
    if (resizing) {
      playClick();
      resizing = false;
      dragWinIdx = -1;
      needFullDraw = true;
      return;
    }
    // Check start button (only if taskbar is visible)
    if (!taskbarHidden && deskPtrHitsStart()) {
      playClick();
      startMenuOpen = !startMenuOpen;
      startMenuSel = 0;
      needFullDraw = true;
    }
    // Check taskbar window buttons (click to focus window)
    else if (!taskbarHidden && !startMenuOpen && mouseY >= SCREEN_HEIGHT - DESK_TASKBAR_H) {
      int ty = SCREEN_HEIGHT - DESK_TASKBAR_H;
      int bx = DESK_START_W + 6;
      for (int8_t i = 0; i < deskWinCount && bx < SCREEN_WIDTH - 40; i++) {
        int8_t idx = deskWinOrder[i];
        if (deskWins[idx].open) {
          // Check if click is on this window's taskbar button
          if (mouseX >= bx && mouseX < bx + 40 && mouseY >= ty + 2 && mouseY < ty + DESK_TASKBAR_H - 2) {
            playClick();
            // Bring window to front (move to position 0 in order)
            if (i > 0) {
              for (int8_t j = i; j > 0; j--) {
                deskWinOrder[j] = deskWinOrder[j - 1];
              }
              deskWinOrder[0] = idx;
            }
            activeWin = idx;
            needFullDraw = true;
            break;
          }
          bx += 42;
        }
      }
    }
    // Check start menu item (only if taskbar is visible)
    // Menu order: Calc(0), Editor(1), Files(2), Music(3), Notes(4), Paint(5), Settings(6), Snake(7), Tetris(8), Timer(9), TODO(10), Sleep(11), Restart(12), Exit(13)
    else if (startMenuOpen && !taskbarHidden) {
      int menuHit = deskPtrHitsStartMenu();
      if (menuHit >= 0) {
        playSelect();
        startMenuOpen = false;
        if (menuHit == 13) {  // Exit
          saveOSData();
          desktopMode = false;
          currentScreen = SCREEN_DRAWER;
          prevSelectedApp = -1;
          tft.fillScreen(COLOR_BG);
          drawMenuBar();
          drawAppDrawer(true);
          return;
        } else if (menuHit == 11) {  // Sleep
          if (dataDirty) { saveOSData(); dataDirty = false; }  // Save before sleep
          sleepMode = true;
          sleepStartTime = millis();
          digitalWrite(TFT_BLK, LOW);
          return;
        } else if (menuHit == 12) {  // Restart
          if (dataDirty) { saveOSData(); dataDirty = false; }  // Save before restart
          ESP.restart();
          return;
        } else if (menuHit == 0) {  // Calc
          deskOpenWindow(1, startMenuItems[menuHit]);
        } else if (menuHit == 1) {  // Editor
          editorText[0] = '\0';
          editorCursor = 0;
          editorFilePath[0] = '\0';
          deskOpenWindow(7, startMenuItems[menuHit]);
        } else if (menuHit == 2) {  // Files
          scanDirectory(currentPath);
          deskOpenWindow(6, startMenuItems[menuHit]);
        } else if (menuHit == 3) {  // Music
          deskOpenWindow(5, startMenuItems[menuHit]);
        } else if (menuHit == 4) {  // Notes
          notesCursor = strlen(notesText);
          deskOpenWindow(4, startMenuItems[menuHit]);
        } else if (menuHit == 5) {  // Paint
          paintInitCanvas();
          deskOpenWindow(10, startMenuItems[menuHit]);
        } else if (menuHit == 6) {  // Settings
          deskOpenWindow(0, startMenuItems[menuHit]);
        } else if (menuHit == 7) {  // Snake
          snakeInitGame();
          deskOpenWindow(11, startMenuItems[menuHit]);
        } else if (menuHit == 8) {  // Tetris
          tetrisInitGame();
          deskOpenWindow(9, startMenuItems[menuHit]);
        } else if (menuHit == 9) {  // Timer
          deskOpenWindow(2, startMenuItems[menuHit]);
        } else if (menuHit == 10) {  // TODO
          deskOpenWindow(3, startMenuItems[menuHit]);
        }
        needFullDraw = true;
      } else {
        // Click outside menu closes it
        startMenuOpen = false;
        needFullDraw = true;
      }
    }
    // Check windows FIRST (before back icon, so windows block clicks)
    else {
      int8_t hitOrder = deskPtrHitsWindow();
      if (hitOrder >= 0) {
        int8_t winIdx = deskWinOrder[hitOrder];
        deskBringToFront(hitOrder);
        activeWin = winIdx;
        
        // Check close button
        if (deskPtrHitsClose(winIdx)) {
          playBack();
          dataDirty = true;  // Mark for deferred save (avoids freeze)
          deskCloseWindow(winIdx);
          activeWin = -1;
          needFullDraw = true;
        }
        // Check maximize button
        else if (deskPtrHitsMaximize(winIdx)) {
          playClick();
          deskWins[winIdx].maximized = !deskWins[winIdx].maximized;
          needFullDraw = true;
        }
        // Check resize corner
        else if (deskPtrHitsResize(winIdx)) {
          playClick();
          resizing = true;
          dragWinIdx = winIdx;
          dragOffX = 8;
          dragOffY = 8;
        }
        // Check title bar for drag
        else if (deskPtrHitsTitleBar(winIdx)) {
          if (!deskWins[winIdx].maximized) {
            playClick();
            dragging = true;
            dragWinIdx = winIdx;
            dragOffX = mouseX - deskWins[winIdx].x;
            dragOffY = mouseY - deskWins[winIdx].y;
          }
        }
        // Click in content area - handle mouse clicks on app elements
        else {
          DeskWindow& w = deskWins[winIdx];
          int16_t wx, wy, ww, wh;
          int cx, cy, cw, ch;
          if (w.fullscreen) {
            wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
            cx = 0; cy = 0; cw = SCREEN_WIDTH; ch = SCREEN_HEIGHT;
          } else if (w.maximized) {
            wx = 0; wy = 0; ww = SCREEN_WIDTH; wh = SCREEN_HEIGHT;
            cx = wx + 2; cy = wy + DESK_TITLEBAR_H + 1;
            cw = ww - 4; ch = wh - DESK_TITLEBAR_H - 3;
          } else {
            wx = w.x; wy = w.y; ww = w.w; wh = w.h;
            cx = wx + 2; cy = wy + DESK_TITLEBAR_H + 1;
            cw = ww - 4; ch = wh - DESK_TITLEBAR_H - 3;
          }
          int lx = mouseX - cx, ly = mouseY - cy;  // Local coords in content area
          
          switch (w.appType) {
            case 0:  // Settings - clickable items with scrollbar
              {
                int itemH = w.maximized ? 14 : 10;
                int scrollW = 8;
                int listH = ch - 4;
                int maxVisible = listH / itemH;
                int sbX = cw - scrollW - 1;
                int vx = w.maximized ? 52 : 36;
                // Check scrollbar
                if (lx >= sbX && lx < sbX + scrollW && ly >= 2 && ly < 2 + listH && SETTINGS_ITEMS > maxVisible) {
                  scrollDragging = true;
                  scrollDragApp = 0;
                  scrollDragVertical = true;
                }
                // Check settings items
                else if (lx < sbX) {
                  int row = ly / itemH;
                  int idx = row + settingsScrollOffset;
                  int smallBtnW = w.maximized ? 14 : 10;
                  if (idx == 0 && lx >= vx) {  // Sound toggle
                    soundEnabled = !soundEnabled;
                    playClick();
                  } else if (idx == 1 && lx >= vx) {  // WiFi - open WiFi connection window
                    playClick();
                    // Open WiFi window for password entry
                    strcpy(wifiEditSSID, "WiFi");  // Placeholder - would scan for networks
                    wifiEditPass[0] = '\0';
                    wifiEditCursor = 0;
                    wifiEditMode = true;
                    deskOpenWindow(8, "WiFi");
                  } else if (idx == 2) {  // Hour +/-
                    if (lx >= vx && lx < vx + smallBtnW) { 
                      hours = (hours + 23) % 24; playTick(); 
                    } else if (lx >= vx + smallBtnW * 2 + 8 && lx < vx + smallBtnW * 3 + 8) { 
                      hours = (hours + 1) % 24; playTick(); 
                    }
                  } else if (idx == 3) {  // Min +/-
                    if (lx >= vx && lx < vx + smallBtnW) { 
                      minutes = (minutes + 59) % 60; playTick(); 
                    } else if (lx >= vx + smallBtnW * 2 + 8 && lx < vx + smallBtnW * 3 + 8) { 
                      minutes = (minutes + 1) % 60; playTick(); 
                    }
                  } else if (idx == 4 && lx >= vx) {  // Boot to Desktop toggle
                    bootToDesktop = !bootToDesktop;
                    playTick();
                    dataDirty = true;  // Use deferred save for speed
                  } else if (idx == 5 && lx >= vx) {  // Wipe
                    prefs.begin("wifi", false);
                    prefs.clear();
                    prefs.end();
                    WiFi.disconnect();
                    wifiConnected = false;
                    wifiSSID[0] = '\0';
                    wifiPass[0] = '\0';
                    // Wipe .osdata folder contents
                    if (sdCardMounted) {
                      SD.remove("/.osdata/notes.txt");
                      SD.remove("/.osdata/todo.dat");
                      SD.remove("/.osdata/settings.dat");
                      SD.remove("/.osdata/wifi.dat");
                      // Clear in-memory data too
                      notesText[0] = '\0';
                      notesCursor = 0;
                      bootToDesktop = false;
                      for (int i = 0; i < MAX_TODO; i++) todoDone[i] = false;
                    }
                    showNotification("Data Wiped!");
                  }
                }
              }
              break;
            case 1:  // Calc - clickable keypad
              {
                int kw = 21, kh = 14;
                int startY = 14;
                if (ly >= startY && ly < startY + 4 * kh) {
                  int row = (ly - startY) / kh;
                  int col = (lx - 2) / kw;
                  if (col >= 0 && col < 4 && row >= 0 && row < 4) {
                    playTick();
                    int key = row * 4 + col;
                    int digits[] = {7, 8, 9, -1, 4, 5, 6, -1, 1, 2, 3, -1, 0, -2, -3, -1};
                    int ops[] = {-1, -1, -1, 1, -1, -1, -1, 2, -1, -1, -1, 3, -1, -1, -1, 4};
                    if (digits[key] >= 0) {
                      if (calcNew) { calcValue = digits[key]; calcNew = false; }
                      else if (calcValue < 100000) calcValue = calcValue * 10 + digits[key];
                    } else if (digits[key] == -2) {
                      calcValue = 0; calcStored = 0; calcOp = 0; calcNew = true;
                    } else if (digits[key] == -3 && calcOp > 0) {
                      switch (calcOp) {
                        case 1: calcValue = calcStored + calcValue; break;
                        case 2: calcValue = calcStored - calcValue; break;
                        case 3: calcValue = calcStored * calcValue; break;
                        case 4: calcValue = calcValue ? calcStored / calcValue : 0; break;
                      }
                      calcOp = 0; calcNew = true;
                    } else if (ops[key] > 0) {
                      calcStored = calcValue; calcOp = ops[key]; calcNew = true;
                    }
                  }
                }
              }
              break;
            case 2:  // Timer - clickable buttons
              {
                int btnY = 18;
                int btn2Y = 30;
                if (ly >= btnY && ly < btnY + 10) {
                  // M-/M+/S-/S+ buttons
                  if (!timerOn && !timerPaused) {
                    if (lx >= 2 && lx < 16 && timerSetMin > 0) { timerSetMin--; playTick(); }
                    else if (lx >= 18 && lx < 32 && timerSetMin < 99) { timerSetMin++; playTick(); }
                    else if (lx >= 34 && lx < 48 && timerSetSec > 0) { timerSetSec--; playTick(); }
                    else if (lx >= 50 && lx < 64 && timerSetSec < 59) { timerSetSec++; playTick(); }
                  }
                } else if (ly >= btn2Y && ly < btn2Y + 12) {
                  // Start/Pause or Reset
                  if (lx >= 2 && lx < 40) {
                    playClick();
                    // Start/Pause/Resume
                    if (!timerOn && !timerPaused) {
                      unsigned long duration = (unsigned long)(timerSetMin * 60 + timerSetSec) * 1000UL;
                      timerEndTime = millis() + duration;
                      timerOn = true;
                      timerPaused = false;
                    } else if (timerOn && !timerPaused) {
                      timerRemaining = timerEndTime - millis();
                      timerPaused = true;
                    } else if (timerPaused) {
                      timerEndTime = millis() + timerRemaining;
                      timerPaused = false;
                    }
                  } else if (lx >= 42 && lx < 72) {
                    playClick();
                    // Reset
                    timerOn = false;
                    timerPaused = false;
                    timerSetMin = 1;
                    timerSetSec = 0;
                  }
                }
              }
              break;
            case 3:  // TODO - clickable checkboxes with keyboard edit
              {
                int itemH = w.maximized ? 14 : 10;
                int scrollW = 8;
                int btnH = w.maximized ? 14 : 10;
                int cbSize = w.maximized ? 10 : 8;
                
                if (deskTodoEditMode && deskTodoEditIdx >= 0) {
                  // === EDIT MODE - handle keyboard clicks ===
                  const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
                  const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
                  const char** rows = (kbMode == 1) ? rowsLower : rowsUpper;
                  int nkw = w.maximized ? 18 : 11;
                  int nkh = w.maximized ? 14 : 11;
                  int editDispH = w.maximized ? 16 : 12;
                  int ky = editDispH + 2;
                  int kbWidth = 10 * (nkw + 1) - 1;
                  int kbOffsetX = (cw - kbWidth) / 2;
                  char* editText = todoItems[deskTodoEditIdx];
                  int editLen = strlen(editText);
                  
                  // Check keyboard rows
                  for (int r = 0; r < 4; r++) {
                    int rowLen = strlen(rows[r]);
                    int rowOffset = kbOffsetX + (10 - rowLen) * (nkw + 1) / 2;
                    if (ly >= ky && ly < ky + nkh) {
                      int col = (lx - rowOffset) / (nkw + 1);
                      if (col >= 0 && col < rowLen && editLen < TODO_ITEM_LEN) {
                        playTick();
                        // Insert character at cursor
                        for (int i = editLen; i > deskTodoCursor; i--) editText[i] = editText[i - 1];
                        editText[deskTodoCursor] = rows[r][col];
                        deskTodoCursor++;
                        editText[editLen + 1] = '\0';
                        dataDirty = true;
                      }
                    }
                    ky += nkh + 1;
                  }
                  
                  // Control buttons row
                  int ctrlBtnW = w.maximized ? 28 : 20;
                  int ctrlBtnX = kbOffsetX;
                  if (ly >= ky && ly < ky + nkh) {
                    // SPACE
                    if (lx >= ctrlBtnX && lx < ctrlBtnX + ctrlBtnW && editLen < TODO_ITEM_LEN) {
                      playTick();
                      for (int i = editLen; i > deskTodoCursor; i--) editText[i] = editText[i - 1];
                      editText[deskTodoCursor] = ' ';
                      deskTodoCursor++;
                      editText[editLen + 1] = '\0';
                      dataDirty = true;
                    }
                    ctrlBtnX += ctrlBtnW + 2;
                    // DEL
                    if (lx >= ctrlBtnX && lx < ctrlBtnX + ctrlBtnW && deskTodoCursor > 0) {
                      playBack();
                      deskTodoCursor--;
                      for (int i = deskTodoCursor; i < editLen; i++) editText[i] = editText[i + 1];
                      dataDirty = true;
                    }
                    ctrlBtnX += ctrlBtnW + 2;
                    // OK (done editing)
                    if (lx >= ctrlBtnX && lx < ctrlBtnX + ctrlBtnW) {
                      playSelect();
                      deskTodoEditMode = false;
                      deskTodoEditIdx = -1;
                      dataDirty = true;
                    }
                    ctrlBtnX += ctrlBtnW + 2;
                    // CASE toggle
                    if (lx >= ctrlBtnX && lx < ctrlBtnX + ctrlBtnW - 4) {
                      playTick();
                      kbMode = kbMode ? 0 : 1;
                    }
                  }
                } else {
                  // === LIST MODE ===
                  int listH = ch - btnH - 4;
                  int maxVisible = listH / itemH;
                  int sbX = cw - scrollW - 1;
                  int delX = cw - scrollW - 12;
                  
                  // Check Add button
                  int addY = listH + 2;
                  if (ly >= addY && ly < addY + btnH && todoCount < MAX_TODO) {
                    playSelect();
                    strcpy(todoItems[todoCount], "New task");
                    todoDone[todoCount] = false;
                    deskTodoEditIdx = todoCount;
                    deskTodoCursor = strlen(todoItems[todoCount]);
                    deskTodoEditMode = true;
                    todoCount++;
                    dataDirty = true;
                  }
                  // Check scrollbar
                  else if (lx >= sbX && lx < sbX + scrollW && ly >= 2 && ly < 2 + listH && todoCount > maxVisible) {
                    scrollDragging = true;
                    scrollDragApp = 3;
                    scrollDragVertical = true;
                  }
                  // Check delete button (X)
                  else if (lx >= delX && lx < delX + 10 && ly >= 2 && ly < listH) {
                    int row = (ly - 2) / itemH;
                    int idx = row + todoScrollOffset;
                    if (idx >= 0 && idx < todoCount && todoCount > 1) {
                      playBack();
                      for (int i = idx; i < todoCount - 1; i++) {
                        strcpy(todoItems[i], todoItems[i + 1]);
                        todoDone[i] = todoDone[i + 1];
                      }
                      todoItems[todoCount - 1][0] = '\0';
                      todoDone[todoCount - 1] = false;
                      todoCount--;
                      dataDirty = true;
                    }
                  }
                  // Check item text (click to edit)
                  else if (lx >= cbSize + 6 && lx < delX) {
                    int row = (ly - 2) / itemH;
                    int idx = row + todoScrollOffset;
                    if (idx >= 0 && idx < todoCount) {
                      playTick();
                      deskTodoEditIdx = idx;
                      deskTodoCursor = strlen(todoItems[idx]);
                      deskTodoEditMode = true;
                    }
                  }
                  // Check checkbox
                  else if (lx >= 2 && lx < cbSize + 4) {
                    int row = (ly - 2) / itemH;
                    int idx = row + todoScrollOffset;
                    if (idx >= 0 && idx < todoCount) {
                      playTick();
                      todoDone[idx] = !todoDone[idx];
                      dataDirty = true;
                    }
                  }
                }
              }
              break;
            case 4:  // Notes - personal notes (saves to .osdata)
              {
                bool largeKB = w.fullscreen || w.maximized;
                const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
                const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
                const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
                const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
                int nkw = largeKB ? 22 : 11;  // Match drawing code
                int nkh = largeKB ? 18 : 11;
                int noteDispH = largeKB ? 20 : 16;
                int ky = noteDispH + 2;
                int noteLen = strlen(notesText);
                int kbWidth = 10 * (nkw + 1) - 1;
                int kbOffsetX = largeKB ? (cw - kbWidth) / 2 : 1;
                
                // Rows 1-2 (10 keys each)
                for (int r = 0; r < 2; r++) {
                  if (ly >= ky && ly < ky + nkh) {
                    int col = (lx - kbOffsetX) / (nkw + 1);
                    if (col >= 0 && col < 10 && noteLen < NOTE_LEN) {
                      playTick();
                      if (notesCursor < noteLen) {
                        for (int i = noteLen; i > notesCursor; i--) notesText[i] = notesText[i - 1];
                      }
                      notesText[notesCursor] = rows[r][col];
                      notesCursor++;
                      notesText[noteLen + 1] = '\0';
                    }
                  }
                  ky += nkh + 1;
                }
                // Row 3 (9 or 10 keys depending on mode)
                int row3Len = (kbMode == 2) ? 10 : 9;
                int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - row3Offset) / (nkw + 1);
                  if (col >= 0 && col < row3Len && noteLen < NOTE_LEN) {
                    playTick();
                    if (notesCursor < noteLen) {
                      for (int i = noteLen; i > notesCursor; i--) notesText[i] = notesText[i - 1];
                    }
                    notesText[notesCursor] = rows[2][col];
                    notesCursor++;
                    notesText[noteLen + 1] = '\0';
                  }
                }
                ky += nkh + 1;
                // Row 4: mode buttons + letters/symbols + DEL
                int row4Len = (kbMode == 2) ? 2 : 7;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - kbOffsetX) / (nkw + 1);
                  if (col == 0) {
                    // aA button - toggle case
                    playClick();
                    kbMode = (kbMode == 0) ? 1 : (kbMode == 1) ? 0 : 0;
                  } else if (col == 1) {
                    // #! button - toggle symbols
                    playClick();
                    kbMode = (kbMode == 2) ? 0 : 2;
                  } else if (col >= 2 && col < 2 + row4Len && noteLen < NOTE_LEN) {
                    playTick();
                    if (notesCursor < noteLen) {
                      for (int i = noteLen; i > notesCursor; i--) notesText[i] = notesText[i - 1];
                    }
                    notesText[notesCursor] = rows[3][col - 2];
                    notesCursor++;
                    if (notesCursor > noteLen) notesText[notesCursor] = '\0';
                    else notesText[noteLen + 1] = '\0';
                  } else if (col == 9 && notesCursor > 0) {
                    // DEL button at position 9
                    playTick();
                    for (int i = notesCursor - 1; i < noteLen; i++) notesText[i] = notesText[i + 1];
                    notesCursor--;
                  }
                }
                ky += nkh + 1;
                // Row 5: Arrow keys + SPACE + CLR + SAVE
                int arrowW = nkw;
                int spaceStartX = kbOffsetX + 2 * (arrowW + 1);
                int spaceW = largeKB ? nkw * 3 : nkw * 2 + 3;
                int clrW = largeKB ? nkw + 8 : nkw * 2;
                int clrX = spaceStartX + spaceW + 2;
                int saveW = largeKB ? nkw * 2 + 8 : nkw * 2 + 4;
                int saveX = clrX + clrW + 2;
                if (ly >= ky && ly < ky + nkh) {
                  if (lx >= kbOffsetX && lx < kbOffsetX + arrowW) {
                    // Left arrow
                    playTick();
                    if (notesCursor > 0) notesCursor--;
                  } else if (lx >= kbOffsetX + arrowW + 1 && lx < kbOffsetX + 2 * arrowW + 1) {
                    // Right arrow
                    playTick();
                    if (notesCursor < noteLen) notesCursor++;
                  } else if (lx >= spaceStartX && lx < spaceStartX + spaceW && noteLen < NOTE_LEN) {
                    playTick();
                    if (notesCursor < noteLen) {
                      for (int i = noteLen; i > notesCursor; i--) notesText[i] = notesText[i - 1];
                    }
                    notesText[notesCursor] = ' ';
                    notesCursor++;
                    notesText[noteLen + 1] = '\0';
                  } else if (lx >= clrX && lx < clrX + clrW) {
                    playClick();
                    notesText[0] = '\0';
                    notesCursor = 0;
                  } else if (lx >= saveX && lx < saveX + saveW && sdCardMounted) {
                    playSelect();
                    saveNotesToOSData();
                  }
                }
              }
              break;
            case 7:  // Editor - for editing .txt files
              {
                const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
                const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
                const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
                const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
                int nkw = w.maximized ? 18 : 11;
                int nkh = w.maximized ? 14 : 11;
                int noteDispH = w.maximized ? 20 : 16;
                int ky = noteDispH + 2;
                int edLen = strlen(editorText);
                int kbWidth = 10 * (nkw + 1) - 1;
                int kbOffsetX = w.maximized ? (cw - kbWidth) / 2 : 1;
                
                // Rows 1-2 (10 keys each)
                for (int r = 0; r < 2; r++) {
                  if (ly >= ky && ly < ky + nkh) {
                    int col = (lx - kbOffsetX) / (nkw + 1);
                    if (col >= 0 && col < 10 && edLen < NOTE_LEN) {
                      playTick();
                      if (editorCursor < edLen) {
                        for (int i = edLen; i > editorCursor; i--) editorText[i] = editorText[i - 1];
                      }
                      editorText[editorCursor] = rows[r][col];
                      editorCursor++;
                      editorText[edLen + 1] = '\0';
                    }
                  }
                  ky += nkh + 1;
                }
                // Row 3 (9 or 10 keys depending on mode)
                int row3Len = (kbMode == 2) ? 10 : 9;
                int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - row3Offset) / (nkw + 1);
                  if (col >= 0 && col < row3Len && edLen < NOTE_LEN) {
                    playTick();
                    if (editorCursor < edLen) {
                      for (int i = edLen; i > editorCursor; i--) editorText[i] = editorText[i - 1];
                    }
                    editorText[editorCursor] = rows[2][col];
                    editorCursor++;
                    editorText[edLen + 1] = '\0';
                  }
                }
                ky += nkh + 1;
                // Row 4: mode buttons + letters/symbols + DEL
                int row4Len = (kbMode == 2) ? 2 : 7;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - kbOffsetX) / (nkw + 1);
                  if (col == 0) {
                    // aA button - toggle case
                    playClick();
                    kbMode = (kbMode == 0) ? 1 : (kbMode == 1) ? 0 : 0;
                  } else if (col == 1) {
                    // #! button - toggle symbols
                    playClick();
                    kbMode = (kbMode == 2) ? 0 : 2;
                  } else if (col >= 2 && col < 2 + row4Len && edLen < NOTE_LEN) {
                    playTick();
                    if (editorCursor < edLen) {
                      for (int i = edLen; i > editorCursor; i--) editorText[i] = editorText[i - 1];
                    }
                    editorText[editorCursor] = rows[3][col - 2];
                    editorCursor++;
                    if (editorCursor > edLen) editorText[editorCursor] = '\0';
                    else editorText[edLen + 1] = '\0';
                  } else if (col == 9 && editorCursor > 0) {
                    // DEL button at position 9
                    playTick();
                    for (int i = editorCursor - 1; i < edLen; i++) editorText[i] = editorText[i + 1];
                    editorCursor--;
                  }
                }
                ky += nkh + 1;
                // Row 5: Arrow keys + SPACE + CANCEL + SAVE
                int arrowW = nkw;
                int spaceStartX = kbOffsetX + 2 * (arrowW + 1);
                int spaceW = w.maximized ? nkw * 2 : nkw * 2;
                int cancelW = w.maximized ? nkw * 2 + 4 : nkw * 2 + 2;
                int cancelX = spaceStartX + spaceW + 2;
                int saveW = w.maximized ? nkw * 2 + 4 : nkw * 2 + 2;
                int saveX = cancelX + cancelW + 2;
                if (ly >= ky && ly < ky + nkh) {
                  if (lx >= kbOffsetX && lx < kbOffsetX + arrowW) {
                    // Left arrow
                    playTick();
                    if (editorCursor > 0) editorCursor--;
                  } else if (lx >= kbOffsetX + arrowW + 1 && lx < kbOffsetX + 2 * arrowW + 1) {
                    // Right arrow
                    playTick();
                    if (editorCursor < edLen) editorCursor++;
                  } else if (lx >= spaceStartX && lx < spaceStartX + spaceW && edLen < NOTE_LEN) {
                    playTick();
                    if (editorCursor < edLen) {
                      for (int i = edLen; i > editorCursor; i--) editorText[i] = editorText[i - 1];
                    }
                    editorText[editorCursor] = ' ';
                    editorCursor++;
                    editorText[edLen + 1] = '\0';
                  } else if (lx >= cancelX && lx < cancelX + cancelW) {
                    playBack();
                    if (renameMode) {
                      cancelRename();
                    } else {
                      editorText[0] = '\0';
                      editorCursor = 0;
                      editorFilePath[0] = '\0';
                    }
                    deskCloseWindow(winIdx);
                  } else if (lx >= saveX && lx < saveX + saveW && sdCardMounted) {
                    playSelect();
                    if (renameMode) {
                      executeRename();
                      deskCloseWindow(winIdx);
                    } else {
                      saveEditorToFile();
                    }
                  }
                }
              }
              break;
            case 8:  // WiFi - dropdown and password entry keyboard
              {
                const char* rowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
                const char* rowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
                const char* rowsSymbol[] = {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./<>?", "`~"};
                const char** rows = (kbMode == 2) ? rowsSymbol : (kbMode == 1) ? rowsLower : rowsUpper;
                int fieldH = w.maximized ? 16 : 14;
                int nkw = w.maximized ? 14 : 11;
                int nkh = w.maximized ? 12 : 11;
                int passLen = strlen(wifiEditPass);
                int kbWidth = 10 * (nkw + 1) - 1;
                int kbOffsetX = (cw - kbWidth) / 2;
                
                // Check dropdown click first (SSID field at top)
                if (ly >= 1 && ly < 1 + fieldH) {
                  playClick();
                  if (!wifiDropdownOpen) {
                    // Open dropdown and start async scan for networks
                    wifiDropdownOpen = true;
                    wifiDropdownScroll = 0;
                    wifiScanSel = 0;
                    wifiScanCount = 0;
                    // Start async WiFi scan
                    WiFi.scanNetworks(true);  // true = async
                    wifiScanning = true;
                  } else {
                    wifiDropdownOpen = false;
                  }
                  break;
                }
                
                // Handle dropdown list clicks when open
                if (wifiDropdownOpen) {
                  int dropY = fieldH + 1;
                  int maxVisible = 4;
                  int dropH = min(wifiScanCount, maxVisible) * fieldH + 2;
                  if (ly >= dropY && ly < dropY + dropH) {
                    int row = (ly - dropY - 1) / fieldH;
                    int idx = row + wifiDropdownScroll;
                    if (idx >= 0 && idx < wifiScanCount) {
                      playClick();
                      strcpy(wifiEditSSID, wifiNetworks[idx]);
                      wifiScanSel = idx;
                      wifiDropdownOpen = false;
                    }
                    break;
                  }
                  // Click outside dropdown closes it
                  wifiDropdownOpen = false;
                  break;
                }
                
                // Keyboard handling (only when dropdown is closed)
                int ky = fieldH * 2 + 4;  // After SSID and password fields
                
                // Rows 1-2 (10 keys each)
                for (int r = 0; r < 2; r++) {
                  if (ly >= ky && ly < ky + nkh) {
                    int col = (lx - kbOffsetX) / (nkw + 1);
                    if (col >= 0 && col < 10 && passLen < 64) {
                      playTick();
                      if (wifiEditCursor < passLen) {
                        for (int i = passLen; i > wifiEditCursor; i--) wifiEditPass[i] = wifiEditPass[i - 1];
                      }
                      wifiEditPass[wifiEditCursor] = rows[r][col];
                      wifiEditCursor++;
                      wifiEditPass[passLen + 1] = '\0';
                    }
                  }
                  ky += nkh + 1;
                }
                // Row 3 (9 or 10 keys depending on mode)
                int row3Len = (kbMode == 2) ? 10 : 9;
                int row3Offset = (kbMode == 2) ? kbOffsetX : kbOffsetX + (nkw + 1) / 2;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - row3Offset) / (nkw + 1);
                  if (col >= 0 && col < row3Len && passLen < 64) {
                    playTick();
                    if (wifiEditCursor < passLen) {
                      for (int i = passLen; i > wifiEditCursor; i--) wifiEditPass[i] = wifiEditPass[i - 1];
                    }
                    wifiEditPass[wifiEditCursor] = rows[2][col];
                    wifiEditCursor++;
                    wifiEditPass[passLen + 1] = '\0';
                  }
                }
                ky += nkh + 1;
                // Row 4: mode buttons + letters/symbols + DEL
                int row4Len = (kbMode == 2) ? 2 : 7;
                if (ly >= ky && ly < ky + nkh) {
                  int col = (lx - kbOffsetX) / (nkw + 1);
                  if (col == 0) {
                    // aA button - toggle case
                    playClick();
                    kbMode = (kbMode == 0) ? 1 : (kbMode == 1) ? 0 : 0;
                  } else if (col == 1) {
                    // #! button - toggle symbols
                    playClick();
                    kbMode = (kbMode == 2) ? 0 : 2;
                  } else if (col >= 2 && col < 2 + row4Len && passLen < 64) {
                    playTick();
                    if (wifiEditCursor < passLen) {
                      for (int i = passLen; i > wifiEditCursor; i--) wifiEditPass[i] = wifiEditPass[i - 1];
                    }
                    wifiEditPass[wifiEditCursor] = rows[3][col - 2];
                    wifiEditCursor++;
                    if (wifiEditCursor > passLen) wifiEditPass[wifiEditCursor] = '\0';
                    else wifiEditPass[passLen + 1] = '\0';
                  } else if (col == 9 && wifiEditCursor > 0) {
                    // DEL button at position 9
                    playTick();
                    for (int i = wifiEditCursor - 1; i < passLen; i++) wifiEditPass[i] = wifiEditPass[i + 1];
                    wifiEditCursor--;
                  }
                }
                ky += nkh + 1;
                // Row 5: Arrow keys + SPACE + X (cancel) + OK (connect)
                int arrowW = nkw;
                int spaceStartX = kbOffsetX + 2 * (arrowW + 1);
                int spaceW = nkw * 2;
                int cancelW = nkw * 2 + 2;
                int cancelX = spaceStartX + spaceW + 2;
                int connectW = nkw * 2 + 2;
                int connectX = cancelX + cancelW + 2;
                if (ly >= ky && ly < ky + nkh) {
                  if (lx >= kbOffsetX && lx < kbOffsetX + arrowW) {
                    // Left arrow
                    playTick();
                    if (wifiEditCursor > 0) wifiEditCursor--;
                  } else if (lx >= kbOffsetX + arrowW + 1 && lx < kbOffsetX + 2 * arrowW + 1) {
                    // Right arrow
                    playTick();
                    if (wifiEditCursor < passLen) wifiEditCursor++;
                  } else if (lx >= spaceStartX && lx < spaceStartX + spaceW && passLen < 64) {
                    playTick();
                    if (wifiEditCursor < passLen) {
                      for (int i = passLen; i > wifiEditCursor; i--) wifiEditPass[i] = wifiEditPass[i - 1];
                    }
                    wifiEditPass[wifiEditCursor] = ' ';
                    wifiEditCursor++;
                    wifiEditPass[passLen + 1] = '\0';
                  } else if (lx >= cancelX && lx < cancelX + cancelW) {
                    playBack();
                    // Cancel - close without connecting
                    wifiEditMode = false;
                    wifiEditPass[0] = '\0';
                    wifiEditSSID[0] = '\0';
                    wifiEditCursor = 0;
                    wifiDropdownOpen = false;
                    deskCloseWindow(winIdx);
                  } else if (lx >= connectX && lx < connectX + connectW) {
                    if (strlen(wifiEditSSID) == 0) {
                      playError();
                      showNotification("Select network");
                    } else {
                      playSelect();
                      // Connect to WiFi (non-blocking)
                      strcpy(wifiSSID, wifiEditSSID);
                      strcpy(wifiPass, wifiEditPass);
                      saveWiFiCredentials();
                      showNotification("Connecting...");
                      wifiEditMode = false;
                      wifiDropdownOpen = false;
                      deskCloseWindow(winIdx);
                      // Start connection attempt (non-blocking)
                      WiFi.begin(wifiSSID, wifiPass);
                      wifiConnecting = true;
                      wifiConnectStart = millis();
                    }
                  }
                }
              }
              break;
            case 5:  // Music - clickable tracks and play button
              {
                int mItemH = w.maximized ? 14 : 10;
                int mBtnH = w.maximized ? 14 : 10;
                int scrollW = 8;
                int listH = ch - mBtnH - 4;
                int maxVisible = listH / mItemH;
                int btnY = ch - mBtnH - 2;
                int sbX = cw - scrollW - 1;
                // Check scrollbar
                if (lx >= sbX && lx < sbX + scrollW && ly >= 2 && ly < 2 + listH && NUM_TRACKS > maxVisible) {
                  scrollDragging = true;
                  scrollDragApp = 5;
                  scrollDragVertical = true;
                }
                // Check play button - play actual songs
                else if (ly >= btnY && lx >= 2 && lx < cw - 2) {
                  if (soundEnabled) {
                    switch (musicTrack) {
                      case 0:  // Twinkle Twinkle
                        playNote(NOTE_C4, 200); playNote(NOTE_C4, 200);
                        playNote(NOTE_G4, 200); playNote(NOTE_G4, 200);
                        playNote(NOTE_A4, 200); playNote(NOTE_A4, 200);
                        playNote(NOTE_G4, 400);
                        playNote(NOTE_F4, 200); playNote(NOTE_F4, 200);
                        playNote(NOTE_E4, 200); playNote(NOTE_E4, 200);
                        playNote(NOTE_D4, 200); playNote(NOTE_D4, 200);
                        playNote(NOTE_C4, 400);
                        break;
                      case 1:  // Happy Birthday
                        playNote(NOTE_C4, 150); playNote(NOTE_C4, 150);
                        playNote(NOTE_D4, 300); playNote(NOTE_C4, 300);
                        playNote(NOTE_F4, 300); playNote(NOTE_E4, 500);
                        playNote(NOTE_C4, 150); playNote(NOTE_C4, 150);
                        playNote(NOTE_D4, 300); playNote(NOTE_C4, 300);
                        playNote(NOTE_G4, 300); playNote(NOTE_F4, 500);
                        break;
                      case 2:  // Fur Elise (simplified)
                        playNote(NOTE_E5, 150); playNote(NOTE_DS4, 150);
                        playNote(NOTE_E5, 150); playNote(NOTE_DS4, 150);
                        playNote(NOTE_E5, 150); playNote(NOTE_B4, 150);
                        playNote(NOTE_D5, 150); playNote(NOTE_C5, 150);
                        playNote(NOTE_A4, 300);
                        break;
                      case 3:  // Scale
                        for (int f = 262; f <= 523; f += 40) {
                          ledcWriteTone(BUZZER_PIN, f);
                          delay(100);
                        }
                        break;
                      case 4:  // Alert
                        for (int i = 0; i < 3; i++) {
                          ledcWriteTone(BUZZER_PIN, 880);
                          delay(100);
                          ledcWriteTone(BUZZER_PIN, 0);
                          delay(50);
                        }
                        break;
                    }
                    ledcWriteTone(BUZZER_PIN, 0);
                  }
                }
                // Track selection
                else if (ly >= 2 && ly < btnY && lx < sbX) {
                  int row = (ly - 2) / mItemH;
                  int idx = row + musicScrollOffset;
                  if (idx >= 0 && idx < NUM_TRACKS) {
                    playTick();
                    musicTrack = idx;
                  }
                }
              }
              break;
            case 6:  // Files - file explorer
              {
                if (!sdCardMounted) break;
                int fItemH = w.maximized ? 14 : 12;
                int pathH = w.maximized ? 14 : 12;
                int btnH = w.maximized ? 14 : 12;
                int maxVisible = (ch - pathH - btnH - 4) / fItemH;
                int listY = pathH + 2;  // After path bar
                int btnY = ch - btnH - 1;
                int btnW = (cw - 12) / 5;
                
                // Handle delete confirmation popup first
                if (deleteConfirmMode) {
                  int popW = cw - 16;
                  int popH = 36;
                  int popX = 8;
                  int popY = ch/2 - popH/2;
                  int btnW2 = (popW - 12) / 2;
                  // Check YES/NO buttons
                  if (ly >= popY + 24 && ly < popY + 34) {
                    if (lx >= popX + 4 && lx < popX + 4 + btnW2) {
                      // YES - actually delete
                      playClick();
                      deleteConfirmMode = false;
                      deleteSelectedFile();
                    } else if (lx >= popX + 8 + btnW2 && lx < popX + 8 + btnW2 * 2) {
                      // NO - cancel
                      playBack();
                      deleteConfirmMode = false;
                    }
                  }
                  break;
                }
                
                // Check buttons first: [/] [UP] [RN] [NW] [DL]
                if (ly >= btnY && ly < btnY + btnH) {
                  if (lx >= 2 && lx < 2 + btnW) {
                    // HOME (/) button - go to root
                    playClick();
                    strcpy(currentPath, "/");
                    scanDirectory(currentPath);
                  } else if (lx >= 4 + btnW && lx < 4 + btnW * 2) {
                    // UP button - go to parent directory
                    playClick();
                    navigateToDir("..");
                  } else if (lx >= 6 + btnW * 2 && lx < 6 + btnW * 3) {
                    // RENAME button
                    playClick();
                    startRenameMode();
                  } else if (lx >= 8 + btnW * 3 && lx < 8 + btnW * 4) {
                    // NEW button - open Editor for new file
                    playClick();
                    editorText[0] = '\0';
                    editorCursor = 0;
                    editorFilePath[0] = '\0';
                    deskOpenWindow(7, "Editor");
                  } else if (lx >= 10 + btnW * 4 && lx < 10 + btnW * 5) {
                    // DEL button - show confirmation
                    playClick();
                    if (fileCount > 0 && fileSelected < fileCount && !fileIsDir[fileSelected]) {
                      deleteConfirmMode = true;
                    } else if (fileIsDir[fileSelected]) {
                      playError();
                      showNotification("Can't del dir");
                    }
                  }
                }
                // Check scrollbar
                int scrollW = 8;
                int sbX = cw - scrollW - 1;
                int listH = ch - pathH - btnH - 4;
                if (lx >= sbX && lx < sbX + scrollW && ly >= listY && ly < listY + listH && fileCount > maxVisible) {
                  // Immediately update scroll position on click
                  int relY = ly - listY;
                  int maxScroll = max(0, fileCount - maxVisible);
                  fileScrollOffset = relY * maxScroll / max(1, listH);
                  if (fileScrollOffset > maxScroll) fileScrollOffset = maxScroll;
                  // Start scrollbar drag for continued dragging
                  scrollDragging = true;
                  scrollDragApp = 6;
                  scrollDragVertical = true;
                  playTick();
                }
                // Check file list
                else if (ly >= listY && ly < btnY && lx < sbX) {
                  int row = (ly - listY) / fItemH;
                  int idx = row + fileScrollOffset;
                  if (idx >= 0 && idx < fileCount) {
                    if (idx == fileSelected) {
                      // Double click on same item - open it
                      playSelect();
                      if (fileIsDir[idx]) {
                        navigateToDir(fileNames[idx]);
                      } else {
                        openFileInViewer(fileNames[idx]);
                      }
                    } else {
                      playTick();
                      fileSelected = idx;
                    }
                  }
                }
              }
              break;
            case 9:  // Tetris - handle menu/game over buttons
              {
                int btnW = 50, btnH = 16;
                int btnX = cw/2 - btnW/2;
                
                if (tetrisShowMenu && !tetrisPlaying) {
                  // Check START button (at y=50)
                  int btnY = 50;
                  if (lx >= btnX && lx < btnX + btnW && ly >= btnY && ly < btnY + btnH) {
                    playSelect();
                    tetrisShowMenu = false;
                    tetrisPlaying = true;
                    tetrisGameOver = false;
                    memset(tetrisBoard, 0, sizeof(tetrisBoard));
                    tetrisScore = 0;
                    tetrisLevel = 1;
                    tetrisLines = 0;
                    tetrisDropInterval = 500;
                    tetrisSoftDropping = false;
                    tetrisLastDrop = millis();
                    tetrisSpawnPiece();
                  }
                } else if (tetrisGameOver) {
                  // Check RETRY button (at y=80)
                  int btnY = 80;
                  if (lx >= btnX && lx < btnX + btnW && ly >= btnY && ly < btnY + btnH) {
                    playSelect();
                    tetrisPlaying = true;
                    tetrisGameOver = false;
                    memset(tetrisBoard, 0, sizeof(tetrisBoard));
                    tetrisScore = 0;
                    tetrisLevel = 1;
                    tetrisLines = 0;
                    tetrisDropInterval = 500;
                    tetrisSoftDropping = false;
                    tetrisLastDrop = millis();
                    tetrisSpawnPiece();
                  }
                  // Check MENU button (at y=100)
                  btnY = 100;
                  if (lx >= btnX && lx < btnX + btnW && ly >= btnY && ly < btnY + btnH) {
                    playClick();
                    tetrisShowMenu = true;
                    tetrisPlaying = false;
                    tetrisGameOver = false;
                  }
                }
              }
              break;
            case 10:  // Paint - mouse-based drawing
              {
                int sidebarW = 20;
                int canvasAreaW = cw - sidebarW - 4;
                int canvasAreaH = ch - 4;
                int cellSize = min(canvasAreaW / PAINT_WIDTH, canvasAreaH / PAINT_HEIGHT);
                if (cellSize < 1) cellSize = 1;
                int canvasW = PAINT_WIDTH * cellSize;
                int canvasH = PAINT_HEIGHT * cellSize;
                int canvasX = 2;
                int canvasY = 2;
                int sbX = cw - sidebarW;
                
                // Check sidebar clicks
                if (lx >= sbX) {
                  int slx = lx - sbX;
                  // Brush button (y: 2-16)
                  if (ly >= 2 && ly < 16 && slx >= 2 && slx < 18) {
                    playClick();
                    paintEraser = false;
                  }
                  // Eraser button (y: 18-32)
                  else if (ly >= 18 && ly < 32 && slx >= 2 && slx < 18) {
                    playClick();
                    paintEraser = true;
                  }
                  // Color palette (y: 36+, 2 columns of 5 rows)
                  else if (ly >= 36 && ly < 36 + 50) {
                    int row = (ly - 36) / 10;
                    int col = (slx - 2) / 9;
                    if (col >= 0 && col < 2 && row >= 0 && row < 5) {
                      int idx = row * 2 + col;
                      if (idx < 10) {
                        playTick();
                        paintColorIdx = idx;
                        paintColor = paintPalette[idx];
                        paintEraser = false;
                      }
                    }
                  }
                  // Save button (bottom)
                  else if (ly >= ch - 14 && ly < ch - 2 && slx >= 2 && slx < 18) {
                    paintSaveToSD();
                  }
                }
                // Check canvas clicks - paint at mouse position
                else if (lx >= canvasX && lx < canvasX + canvasW &&
                         ly >= canvasY && ly < canvasY + canvasH) {
                  int px = (lx - canvasX) / cellSize;
                  int py = (ly - canvasY) / cellSize;
                  if (px >= 0 && px < PAINT_WIDTH && py >= 0 && py < PAINT_HEIGHT) {
                    if (paintEraser) {
                      paintCanvas[py][px] = 0xFFFF;  // Erase to white
                    } else {
                      paintCanvas[py][px] = paintColor;
                    }
                    playTick();
                  }
                }
              }
              break;
            case 12:  // Image viewer - handle close button overlay click
              {
                // Close button is in top-right corner when overlay is visible
                if (imgViewerOverlayVisible && bmpViewBuffer) {
                  int btnSize = 16;
                  int btnX = cw - btnSize - 4;
                  int btnY = 4;
                  if (lx >= btnX && lx < btnX + btnSize && ly >= btnY && ly < btnY + btnSize) {
                    // Close button clicked
                    playBack();
                    deskCloseWindow(winIdx);
                    activeWin = -1;
                  }
                }
                // Any click shows the overlay again
                imgViewerOverlayVisible = true;
                imgViewerOverlayTime = millis();
              }
              break;
          }
          needFullDraw = true;
        }
      } else {
        // No window hit - check desktop shortcut icon (double-click to open)
        if (sdCardMounted) {
          int iconX = 4;
          int iconY = 4;
          if (mouseX >= iconX && mouseX < iconX + 28 && mouseY >= iconY && mouseY < iconY + 32) {
            // Check for double-click (within 500ms and same location)
            if (now - lastDesktopClick < 500 && 
                abs(mouseX - lastClickX) < 10 && abs(mouseY - lastClickY) < 10) {
              // Double-click - open Files
              playSelect();
              scanDirectory(currentPath);
              deskOpenWindow(6, "Files");
              needFullDraw = true;
              lastDesktopClick = 0;  // Reset
            } else {
              lastDesktopClick = now;
              lastClickX = mouseX;
              lastClickY = mouseY;
            }
          }
        }
      }
    }
  }
  
  // Both dragging and resizing now use toggle mode - click to start, click again to stop
  
  // Redraw logic
  if (needFullDraw) {
    deskFullRedraw();
    needFullDraw = false;
  } else if (mouseX != prevMouseX || mouseY != prevMouseY) {
    // With double buffering, just do a full redraw - it's flicker-free
    deskFullRedraw();
  }
}

// ===== BACKGROUND TIMER =====
void updateBackgroundTimer() {
  if (!timerOn || timerPaused) return;
  
  unsigned long now = millis();
  
  // Check if timer finished
  if (now >= timerEndTime) {
    timerOn = false;
    timerPaused = false;
    playAlert();
    showNotification("Timer Done!");
    
    // Force redraw if viewing timer
    if (currentScreen == SCREEN_APP && selectedApp == APP_TIMER) {
      lastButton = 0;
      drawTimerApp();
    }
  }
  
  // Update menu bar timer display if not in timer app
  if (timerOn && currentScreen != SCREEN_HOME && 
      !(currentScreen == SCREEN_APP && selectedApp == APP_TIMER)) {
    static unsigned long lastMenuTimer = 0;
    if (now - lastMenuTimer >= 1000) {
      lastMenuTimer = now;
      // Draw timer in menu bar
      long remaining = (timerEndTime - now) / 1000;
      int m = remaining / 60;
      int s = remaining % 60;
      tft.fillRect(170, 2, 65, 14, COLOR_BG);
      tft.setTextSize(1);
      tft.setTextColor(COLOR_TIMER);
      tft.setCursor(172, 4);
      if (m < 10) tft.print("0");
      tft.print(m);
      tft.print(":");
      if (s < 10) tft.print("0");
      tft.print(s);
    }
  }
}

// ===== NOTIFICATION SYSTEM =====
void showNotification(const char* text) {
  playPopup();  // Sound feedback for notifications
  if (desktopMode) {
    // Desktop mode: use popup dialog
    strncpy(notificationMsg, text, 31);
    notificationMsg[31] = '\0';
    notificationPopup = true;
  } else {
    // Normal mode: use banner notification
    strncpy(notifText, text, 19);
    notifText[19] = '\0';
    notifActive = true;
    notifTime = millis();
    drawNotification();
  }
}

void dismissNotification() {
  notifActive = false;
  // Redraw current screen to remove notification
  if (currentScreen == SCREEN_HOME) {
    drawHomeScreen();
  } else if (currentScreen == SCREEN_DRAWER) {
    tft.fillScreen(COLOR_BG);
    drawMenuBar();
    drawAppDrawer(true);
  } else if (currentScreen == SCREEN_APP) {
    tft.fillScreen(COLOR_BG);
    drawMenuBar();
    runApp();
  } else if (currentScreen == SCREEN_SWITCHER) {
    tft.fillScreen(COLOR_BG);
    drawAppSwitcher();
  }
}

void drawNotification() {
  // Draw notification banner at very top (above everything including menu bar)
  tft.fillRoundRect(10, 5, 220, 40, 8, 0x4208);  // Dark gray bg
  tft.drawRoundRect(10, 5, 220, 40, 8, COLOR_ACCENT);
  
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  
  // Center text
  int textLen = strlen(notifText);
  int textX = 120 - (textLen * 6);
  tft.setCursor(textX, 12);
  tft.print(notifText);
  
  tft.setTextSize(1);
  tft.setTextColor(0x6B6D);
  tft.setCursor(70, 32);
  tft.print("Hold MID to dismiss");
}

// ===== APP SWITCHER =====
void drawAppSwitcher() {
  // Use double buffer for consistent rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  g->fillScreen(COLOR_BG);
  
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(60, 5);
  g->print("OPEN APPS");
  
  int y = 28;
  int count = 0;
  
  for (int i = 0; i < NUM_APPS; i++) {
    if (appOpen[i] || (i == APP_TIMER && (timerOn || timerPaused))) {
      bool sel = (i == switcherSel);
      
      if (sel) {
        g->fillRect(0, y, SCREEN_WIDTH, 26, COLOR_HIGHLIGHT);
      }
      
      // App icon (small) - Desktop gets gray instead of blue
      uint16_t iconBg = (i == APP_DESKTOP) ? 0x4208 : appColors[i];
      g->fillRoundRect(8, y + 3, 20, 20, 3, iconBg);
      
      // App name
      g->setTextSize(2);
      g->setTextColor(sel ? COLOR_ACCENT : COLOR_TEXT);
      g->setCursor(35, y + 5);
      g->print(appNames[i]);
      
      // Show timer status
      if (i == APP_TIMER && timerOn) {
        g->setTextSize(1);
        g->setTextColor(COLOR_TIMER);
        long rem = (timerEndTime - millis()) / 1000;
        g->setCursor(180, y + 8);
        g->print(rem / 60);
        g->print(":");
        if (rem % 60 < 10) g->print("0");
        g->print(rem % 60);
      }
      
      y += 28;
      count++;
    }
  }
  
  if (count == 0) {
    g->setTextSize(2);
    g->setTextColor(0x6B6D);
    g->setCursor(50, 70);
    g->print("No open apps");
  }
  
  g->setTextSize(1);
  g->setTextColor(0x6B6D);
  g->setCursor(60, SCREEN_HEIGHT - 10);
  g->print("RST to close app");
  
  // Flush buffer to screen
  flushBuffer();
  
  // Redraw notification on top if active (directly to tft, on top of buffer)
  if (notifActive) drawNotification();
}

// ===== WIFI FUNCTIONS =====
void loadWiFiCredentials() {
  if (!sdCardMounted) return;
  ensureOSDataFolder();
  
  File f = SD.open("/.osdata/wifi.dat", FILE_READ);
  if (f) {
    String line = f.readStringUntil('\n');
    line.trim();
    line.toCharArray(wifiSSID, 33);
    line = f.readStringUntil('\n');
    line.trim();
    line.toCharArray(wifiPass, 65);
    f.close();
  }
  // CRITICAL: Restore TFT state after SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
}

void saveWiFiCredentials() {
  if (!sdCardMounted) return;
  ensureOSDataFolder();
  
  File f = SD.open("/.osdata/wifi.dat", FILE_WRITE);
  if (f) {
    f.println(wifiSSID);
    f.println(wifiPass);
    f.close();
  }
  // CRITICAL: Restore TFT state after SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
}

// ===== OS DATA PERSISTENCE (SD card .osdata folder) =====
#define OSDATA_PATH "/.osdata"

void ensureOSDataFolder() {
  if (!sdCardMounted) return;
  if (!SD.exists(OSDATA_PATH)) {
    SD.mkdir(OSDATA_PATH);
  }
}

void loadOSData() {
  if (!sdCardMounted) return;
  stopSound();  // Stop any buzzer before slow SD operations
  ensureOSDataFolder();
  
  // Load notes
  File f = SD.open("/.osdata/notes.txt", FILE_READ);
  if (f) {
    int i = 0;
    while (f.available() && i < NOTE_LEN) {
      char c = f.read();
      if (c >= 32 && c < 127) notesText[i++] = c;
    }
    notesText[i] = '\0';
    f.close();
  }
  
  // Load TODO states, count, and items
  f = SD.open("/.osdata/todo.dat", FILE_READ);
  if (f) {
    // First line: count
    String countLine = f.readStringUntil('\n');
    todoCount = countLine.toInt();
    if (todoCount < 1) todoCount = 1;
    if (todoCount > MAX_TODO) todoCount = MAX_TODO;
    
    // Second line: done states
    String doneLine = f.readStringUntil('\n');
    for (int i = 0; i < todoCount && i < (int)doneLine.length(); i++) {
      todoDone[i] = (doneLine[i] == '1');
    }
    
    // Remaining lines: item text
    for (int i = 0; i < todoCount && f.available(); i++) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) {
        strncpy(todoItems[i], line.c_str(), TODO_ITEM_LEN);
        todoItems[i][TODO_ITEM_LEN] = '\0';
      }
    }
    f.close();
  }
  
  // Load settings (calc, music, timer, bootDesk)
  f = SD.open("/.osdata/settings.dat", FILE_READ);
  if (f) {
    String line;
    while (f.available()) {
      line = f.readStringUntil('\n');
      if (line.startsWith("calc=")) calcValue = line.substring(5).toInt();
      else if (line.startsWith("music=")) musicTrack = line.substring(6).toInt();
      else if (line.startsWith("timerMin=")) timerSetMin = line.substring(9).toInt();
      else if (line.startsWith("timerSec=")) timerSetSec = line.substring(9).toInt();
      else if (line.startsWith("bootDesk=")) bootToDesktop = (line.substring(9).toInt() == 1);
      else if (line.startsWith("tetrisHi=")) tetrisHighScore = line.substring(9).toInt();
      else if (line.startsWith("snakeHi=")) snakeHighScore = line.substring(8).toInt();
    }
    f.close();
  }
  
  // CRITICAL: Restore TFT state after all SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
}

void saveOSData() {
  if (!sdCardMounted) return;
  stopSound();  // Stop any buzzer immediately before slow SD operations
  ensureOSDataFolder();
  
  // Save notes
  File f = SD.open("/.osdata/notes.txt", FILE_WRITE);
  if (f) {
    f.print(notesText);
    f.close();
  }
  
  // Save TODO count, states, and items
  f = SD.open("/.osdata/todo.dat", FILE_WRITE);
  if (f) {
    // First line: count
    f.println(todoCount);
    // Second line: done states
    for (int i = 0; i < todoCount; i++) {
      f.print(todoDone[i] ? '1' : '0');
    }
    f.println();
    // Remaining lines: item text
    for (int i = 0; i < todoCount; i++) {
      f.println(todoItems[i]);
    }
    f.close();
  }
  
  // Save settings
  f = SD.open("/.osdata/settings.dat", FILE_WRITE);
  if (f) {
    f.print("calc="); f.println(calcValue);
    f.print("music="); f.println(musicTrack);
    f.print("timerMin="); f.println(timerSetMin);
    f.print("timerSec="); f.println(timerSetSec);
    f.print("bootDesk="); f.println(bootToDesktop ? 1 : 0);
    f.print("tetrisHi="); f.println(tetrisHighScore);
    f.print("snakeHi="); f.println(snakeHighScore);
    f.close();
  }
  
  // CRITICAL: Restore TFT state after all SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
}

void saveNotesToOSData() {
  if (!sdCardMounted) return;
  ensureOSDataFolder();
  File f = SD.open("/.osdata/notes.txt", FILE_WRITE);
  if (f) {
    f.print(notesText);
    f.close();
    // Restore TFT state after SD operations
    digitalWrite(SD_CS, HIGH);
    delayMicroseconds(100);
    ensureTFTState();
    showNotification("Notes saved!");
  } else {
    showNotification("Save failed");
  }
}

// Take a screenshot and save as BMP to SD card root
// Works in both desktop and non-desktop modes
void takeScreenshot() {
  // Ensure we have a frame buffer
  if (!initDoubleBuffer()) {
    showNotification("No memory!");
    return;
  }
  
  // Verify buffer is valid
  if (!deskBuffer || !deskBuffer->getBuffer()) {
    showNotification("Buffer invalid!");
    return;
  }
  
  // Force a complete redraw to ensure buffer has current screen
  if (desktopMode) {
    // Desktop mode - force full redraw to buffer
    // Ensure gfx points to deskBuffer
    gfx = (Adafruit_GFX*)deskBuffer;
    deskFullRedraw();
    // DON'T flush here - we want to capture what's in the buffer
  }
  // Non-desktop mode: deskBuffer already contains the screen content
  // since all non-desktop apps now render to deskBuffer and call flushBuffer()
  
  // Get buffer pointer
  uint16_t* srcBuffer = deskBuffer->getBuffer();
  if (!srcBuffer) {
    showNotification("Buffer error!");
    return;
  }
  
  int width = SCREEN_WIDTH;   // 240
  int height = SCREEN_HEIGHT; // 135
  
  // Reuse bmpViewBuffer for screenshot copy to avoid memory fragmentation
  // Allocate it if not already allocated - uses PSRAM if available, else SRAM
  if (!bmpViewBuffer) {
    bmpViewBuffer = (uint16_t*)psram_malloc(width * height * sizeof(uint16_t));
  }
  if (!bmpViewBuffer) {
    showNotification("No memory!");
    return;
  }
  
  // Copy buffer content to bmpViewBuffer
  memcpy(bmpViewBuffer, srcBuffer, width * height * sizeof(uint16_t));
  uint16_t* bufferCopy = bmpViewBuffer;  // Use same pointer for rest of function
  
  if (!ensureSDCard()) {
    showNotification("No SD card!");
    return;
  }
  
  // Generate filename - find next available number (1-indexed)
  char filename[32];
  int num = 1;
  while (num < 10000) {  // Support up to 9999 screenshots
    snprintf(filename, 32, "/screenshot%d.bmp", num);
    if (!SD.exists(filename)) break;
    num++;
  }
  screenshotNum = num;  // Update for reference
  
  File f = SD.open(filename, FILE_WRITE);
  if (!f) {
    showNotification("Save failed");
    return;
  }
  
  // Use same BMP format as Paint app (which works correctly)
  // 24-bit RGB, top-down (negative height)
  int padding = (4 - (width * 3) % 4) % 4;
  uint32_t rowSizePadded = width * 3 + padding;
  uint32_t imageSize = rowSizePadded * height;
  uint32_t fileSize = 54 + imageSize;
  uint32_t dataOffset = 54;
  uint32_t headerSize = 40;
  int32_t bmpWidth = width;
  int32_t bmpHeight = -height;  // Negative = top-down (like Paint)
  uint16_t planes = 1;
  uint16_t bpp = 24;
  uint32_t compression = 0;
  uint32_t zero = 0;
  
  // Write BMP header (same method as Paint)
  f.write('B'); f.write('M');
  f.write((uint8_t*)&fileSize, 4);
  f.write((uint8_t*)&zero, 4);  // Reserved
  f.write((uint8_t*)&dataOffset, 4);
  
  // DIB header
  f.write((uint8_t*)&headerSize, 4);
  f.write((uint8_t*)&bmpWidth, 4);
  f.write((uint8_t*)&bmpHeight, 4);
  f.write((uint8_t*)&planes, 2);
  f.write((uint8_t*)&bpp, 2);
  f.write((uint8_t*)&compression, 4);
  f.write((uint8_t*)&imageSize, 4);
  f.write((uint8_t*)&zero, 4);  // X pixels per meter
  f.write((uint8_t*)&zero, 4);  // Y pixels per meter
  f.write((uint8_t*)&zero, 4);  // Colors used
  f.write((uint8_t*)&zero, 4);  // Important colors
  
  // Write pixel data (top-down, same as Paint)
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uint16_t c = bufferCopy[y * width + x];
      // Convert RGB565 to RGB888 (same as Paint)
      uint8_t r = ((c >> 11) & 0x1F) << 3;
      uint8_t g = ((c >> 5) & 0x3F) << 2;
      uint8_t b = (c & 0x1F) << 3;
      // BMP stores BGR
      f.write(b);
      f.write(g);
      f.write(r);
    }
    // Padding to 4-byte boundary
    for (int p = 0; p < padding; p++) f.write((uint8_t)0);
    yield();
  }
  
  f.close();
  
  // Note: bufferCopy points to bmpViewBuffer which is kept allocated
  
  // Restore TFT after SD operations
  digitalWrite(SD_CS, HIGH);
  delay(10);
  tft.setSPISpeed(20000000);
  
  // Show notification/popup
  showNotification("Screenshot saved!");
  
  // Set timestamp for delayed file refresh (0.5s later)
  screenshotTakenTime = millis();
  
  // Force immediate display
  if (desktopMode) {
    // Desktop mode: force full redraw to show popup dialog
    deskNeedRedraw = true;
  } else {
    // Non-desktop: draw notification banner directly
    drawNotification();
  }
}

// ===== TETRIS GAME FUNCTIONS =====
void tetrisInitGame() {
  memset(tetrisBoard, 0, sizeof(tetrisBoard));
  tetrisScore = 0;
  tetrisLevel = 1;
  tetrisLines = 0;
  tetrisGameOver = false;
  tetrisPlaying = false;
  tetrisShowMenu = true;
  tetrisPaused = false;
  tetrisMenuSel = 0;
  tetrisDropInterval = 500;
  tetrisSoftDropInterval = 50;  // Fast drop when holding down
  tetrisSoftDropping = false;
  tetrisLastDrop = millis();
}

void tetrisSpawnPiece() {
  tetrisPieceType = random(7);
  tetrisPieceRot = 0;
  tetrisPieceX = TETRIS_COLS / 2 - 2;
  tetrisPieceY = 0;
  
  // Check if spawn position is blocked (game over)
  if (tetrisCheckCollision(tetrisPieceX, tetrisPieceY, tetrisPieceRot)) {
    tetrisGameOver = true;
    tetrisPlaying = false;
    if (tetrisScore > tetrisHighScore) {
      tetrisHighScore = tetrisScore;
      saveOSData();
    }
  }
}

bool tetrisCheckCollision(int px, int py, int rot) {
  uint16_t piece = tetrisPieces[tetrisPieceType][rot];
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (piece & (0x8000 >> (r * 4 + c))) {
        int br = py + r;
        int bc = px + c;
        if (bc < 0 || bc >= TETRIS_COLS || br >= TETRIS_ROWS) return true;
        if (br >= 0 && tetrisBoard[br][bc] > 0) return true;
      }
    }
  }
  return false;
}

void tetrisLockPiece() {
  uint16_t piece = tetrisPieces[tetrisPieceType][tetrisPieceRot];
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if (piece & (0x8000 >> (r * 4 + c))) {
        int br = tetrisPieceY + r;
        int bc = tetrisPieceX + c;
        if (br >= 0 && br < TETRIS_ROWS && bc >= 0 && bc < TETRIS_COLS) {
          tetrisBoard[br][bc] = tetrisPieceType + 1;
        }
      }
    }
  }
  tetrisClearLines();
  tetrisSpawnPiece();
}

void tetrisClearLines() {
  int cleared = 0;
  for (int r = TETRIS_ROWS - 1; r >= 0; r--) {
    bool full = true;
    for (int c = 0; c < TETRIS_COLS; c++) {
      if (tetrisBoard[r][c] == 0) { full = false; break; }
    }
    if (full) {
      cleared++;
      // Move all rows above down
      for (int rr = r; rr > 0; rr--) {
        for (int c = 0; c < TETRIS_COLS; c++) {
          tetrisBoard[rr][c] = tetrisBoard[rr - 1][c];
        }
      }
      for (int c = 0; c < TETRIS_COLS; c++) tetrisBoard[0][c] = 0;
      r++;  // Check this row again
    }
  }
  if (cleared > 0) {
    tetrisLines += cleared;
    int points[] = {0, 100, 300, 500, 800};
    tetrisScore += points[cleared] * tetrisLevel;
    tetrisLevel = 1 + tetrisLines / 10;
    tetrisDropInterval = max(100, 500 - (tetrisLevel - 1) * 50);
    // Use quiet tone to avoid interrupting music
    if (soundEnabled) playQuietTone(600, 30);  // Higher pitch for line clear
  }
}

void tetrisMove(int dx) {
  if (!tetrisCheckCollision(tetrisPieceX + dx, tetrisPieceY, tetrisPieceRot)) {
    tetrisPieceX += dx;
  }
}

void tetrisRotate() {
  int newRot = (tetrisPieceRot + 1) % 4;
  if (!tetrisCheckCollision(tetrisPieceX, tetrisPieceY, newRot)) {
    tetrisPieceRot = newRot;
  }
}

void tetrisDrop() {
  if (!tetrisCheckCollision(tetrisPieceX, tetrisPieceY + 1, tetrisPieceRot)) {
    tetrisPieceY++;
  } else {
    tetrisLockPiece();
  }
}

void tetrisHardDrop() {
  while (!tetrisCheckCollision(tetrisPieceX, tetrisPieceY + 1, tetrisPieceRot)) {
    tetrisPieceY++;
    tetrisScore += 2;
  }
  tetrisLockPiece();
}

void tetrisUpdate() {
  if (!tetrisPlaying || tetrisGameOver || tetrisPaused) return;
  
  unsigned long now = millis();
  int dropTime = tetrisSoftDropping ? tetrisSoftDropInterval : tetrisDropInterval;
  if (now - tetrisLastDrop >= (unsigned long)dropTime) {
    tetrisLastDrop = now;
    tetrisDrop();
    if (tetrisSoftDropping) tetrisScore += 1;  // Bonus for soft drop
  }
}

// Play a quieter tone using lower duty cycle (0-255, lower = quieter)
void playQuietTone(uint16_t freq, uint8_t volume) {
  if (freq == 0) {
    ledcWrite(BUZZER_PIN, 0);
  } else {
    ledcWriteTone(BUZZER_PIN, freq);
    ledcWrite(BUZZER_PIN, volume);  // Reduce duty cycle for quieter sound
  }
}

// Update Tetris background music - call from main loop
void tetrisMusicUpdate() {
  // Only play when Tetris is active and playing (not paused, not game over, not in menu)
  bool shouldPlay = (activeWin >= 0 && deskWins[activeWin].appType == 9 && 
                     deskWins[activeWin].fullscreen && tetrisPlaying && 
                     !tetrisPaused && !tetrisGameOver);
  
  if (!shouldPlay) {
    if (tetrisMusicPlaying) {
      ledcWrite(BUZZER_PIN, 0);  // Stop sound
      tetrisMusicPlaying = false;
    }
    return;
  }
  
  // Start music if not playing
  if (!tetrisMusicPlaying) {
    tetrisMusicPlaying = true;
    tetrisMusicNote = 0;
    tetrisMusicLastNote = millis();
    // Play first note
    if (tetrisTheme[0][0] > 0) {
      playQuietTone(tetrisTheme[0][0], 15);
    }
  }
  
  unsigned long now = millis();
  uint16_t duration = tetrisTheme[tetrisMusicNote][1];
  
  // Time for next note?
  if (now - tetrisMusicLastNote >= duration) {
    tetrisMusicNote++;
    if (tetrisMusicNote >= TETRIS_THEME_LEN) {
      tetrisMusicNote = 0;  // Loop
    }
    tetrisMusicLastNote = now;
    
    uint16_t freq = tetrisTheme[tetrisMusicNote][0];
    playQuietTone(freq, 15);  // Volume 15 out of 255 (much quieter)
  }
}

// Update Snake background music - call from main loop
void snakeMusicUpdate() {
  // Only play when Snake is active and playing (not paused, not game over, not in menu)
  bool shouldPlay = (activeWin >= 0 && deskWins[activeWin].appType == 11 && 
                     deskWins[activeWin].fullscreen && snakePlaying && 
                     !snakePaused && !snakeGameOver);
  
  if (!shouldPlay) {
    if (snakeMusicPlaying) {
      ledcWrite(BUZZER_PIN, 0);  // Stop sound
      snakeMusicPlaying = false;
    }
    return;
  }
  
  // Start music if not playing
  if (!snakeMusicPlaying) {
    snakeMusicPlaying = true;
    snakeMusicNote = 0;
    snakeMusicLastNote = millis();
    if (snakeTheme[0][0] > 0) {
      playQuietTone(snakeTheme[0][0], 15);
    }
  }
  
  unsigned long now = millis();
  uint16_t duration = snakeTheme[snakeMusicNote][1];
  
  // Time for next note?
  if (now - snakeMusicLastNote >= duration) {
    snakeMusicNote++;
    if (snakeMusicNote >= SNAKE_THEME_LEN) {
      snakeMusicNote = 0;  // Loop
    }
    snakeMusicLastNote = now;
    
    uint16_t freq = snakeTheme[snakeMusicNote][0];
    playQuietTone(freq, 15);
  }
}

void saveEditorToFile() {
  if (!sdCardMounted) return;
  
  // If no file path, create new file
  if (strlen(editorFilePath) == 0) {
    // Generate filename from first few chars or use default
    char filename[MAX_PATH_LEN];
    snprintf(filename, MAX_PATH_LEN, "%s/NEW%lu.TXT", currentPath, millis() % 10000);
    strcpy(editorFilePath, filename);
  }
  
  File f = SD.open(editorFilePath, FILE_WRITE);
  if (f) {
    f.print(editorText);
    f.close();
    // CRITICAL: Restore TFT state after SD operations
    digitalWrite(SD_CS, HIGH);
    delayMicroseconds(100);
    ensureTFTState();
    showNotification("File saved!");
    scanDirectory(currentPath);
  } else {
    showNotification("Save failed");
  }
}

void connectWiFi(bool drawProgress) {
  if (strlen(wifiSSID) == 0) return;
  stopSound();  // Stop any buzzer before slow WiFi operations
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPass);
 
  unsigned long start = millis();
  unsigned long lastDot = 0;
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 30000) {
    delay(50);
    yield();
    if (drawProgress && millis() - lastDot > 400) {
      lastDot = millis();
      dots = (dots + 1) % 4;
      tft.fillRect(40, 80, 160, 18, COLOR_BG);
      tft.setTextSize(2);
      tft.setTextColor(COLOR_TEXT);
      tft.setCursor(40, 60);
      tft.print("Connecting");
      for (int i = 0; i < dots; i++) tft.print(".");
    }
  }
 
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    syncTimeFromNTP();
  }
}

void syncTimeFromNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  // Non-blocking - time will sync in background
  // Check is done in main loop via ntpSyncing flag
}

// ===== SD CARD FILE FUNCTIONS =====

// Quick file count check - returns count without reading names (non-blocking friendly)
int quickFileCount(const char* path) {
  if (!sdCardMounted) return -1;
  
  File root = SD.open(path);
  if (!root || !root.isDirectory()) {
    root.close();
    digitalWrite(SD_CS, HIGH);
    return -1;
  }
  
  int count = 0;
  File file = root.openNextFile();
  while (file && count < MAX_FILES) {
    const char* name = file.name();
    const char* lastSlash = strrchr(name, '/');
    if (lastSlash) name = lastSlash + 1;
    if (name[0] != '.' && strlen(name) > 0) {
      count++;
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  
  // Restore TFT state
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(50);
  ensureTFTState();
  return count;
}

// Natural sort comparison - alphabetical with numeric sequences sorted numerically
// e.g., "file1", "file2", "file10" instead of "file1", "file10", "file2"
int naturalCompare(const char* a, const char* b) {
  while (*a && *b) {
    // Both are digits - compare numerically
    if (isdigit(*a) && isdigit(*b)) {
      // Extract numbers
      long numA = 0, numB = 0;
      while (isdigit(*a)) { numA = numA * 10 + (*a - '0'); a++; }
      while (isdigit(*b)) { numB = numB * 10 + (*b - '0'); b++; }
      if (numA != numB) return (numA < numB) ? -1 : 1;
    } else {
      // Compare characters case-insensitively
      char ca = tolower(*a);
      char cb = tolower(*b);
      if (ca != cb) return (ca < cb) ? -1 : 1;
      a++; b++;
    }
  }
  // Shorter string comes first
  if (*a) return 1;
  if (*b) return -1;
  return 0;
}

// Sort files: directories first, then alphabetically with natural number ordering
void sortFiles() {
  // Simple bubble sort (file count is small, typically < 50)
  for (int i = 0; i < fileCount - 1; i++) {
    for (int j = 0; j < fileCount - i - 1; j++) {
      bool swap = false;
      // Directories come before files
      if (!fileIsDir[j] && fileIsDir[j + 1]) {
        swap = true;
      } else if (fileIsDir[j] == fileIsDir[j + 1]) {
        // Same type - compare names naturally
        if (naturalCompare(fileNames[j], fileNames[j + 1]) > 0) {
          swap = true;
        }
      }
      if (swap) {
        // Swap names
        char tempName[MAX_FILENAME_LEN];
        strcpy(tempName, fileNames[j]);
        strcpy(fileNames[j], fileNames[j + 1]);
        strcpy(fileNames[j + 1], tempName);
        // Swap isDir flags
        bool tempDir = fileIsDir[j];
        fileIsDir[j] = fileIsDir[j + 1];
        fileIsDir[j + 1] = tempDir;
      }
    }
  }
}

void scanDirectory(const char* path) {
  fileCount = 0;
  fileScrollOffset = 0;
  fileSelected = 0;
  
  if (!sdCardMounted) return;
  
  File root = SD.open(path);
  if (!root || !root.isDirectory()) {
    root.close();
    return;
  }
  
  File file = root.openNextFile();
  while (file && fileCount < MAX_FILES) {
    const char* name = file.name();
    // ESP32 SD library returns full path - extract just filename
    const char* lastSlash = strrchr(name, '/');
    if (lastSlash) name = lastSlash + 1;
    // Skip hidden files
    if (name[0] != '.' && strlen(name) > 0) {
      strncpy(fileNames[fileCount], name, MAX_FILENAME_LEN - 1);
      fileNames[fileCount][MAX_FILENAME_LEN - 1] = '\0';
      fileIsDir[fileCount] = file.isDirectory();
      fileCount++;
    }
    file = root.openNextFile();
  }
  root.close();
  
  // Sort files alphabetically with natural number ordering
  sortFiles();
  
  // CRITICAL: Restore TFT state after SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
}

void navigateToDir(const char* dirName) {
  if (strcmp(dirName, "..") == 0) {
    // Go up one level
    int len = strlen(currentPath);
    if (len > 1) {
      // Remove trailing slash if present
      if (currentPath[len - 1] == '/') currentPath[len - 1] = '\0';
      // Find last slash
      char* lastSlash = strrchr(currentPath, '/');
      if (lastSlash && lastSlash != currentPath) {
        *lastSlash = '\0';
      } else {
        strcpy(currentPath, "/");
      }
    }
  } else {
    // Go into directory
    int len = strlen(currentPath);
    if (len > 1) {
      strncat(currentPath, "/", MAX_PATH_LEN - len - 1);
    }
    strncat(currentPath, dirName, MAX_PATH_LEN - strlen(currentPath) - 1);
  }
  scanDirectory(currentPath);
}

bool isImageFile(const char* filename) {
  const char* ext = strrchr(filename, '.');
  if (!ext) return false;
  return (strcasecmp(ext, ".bmp") == 0);  // Only BMP supported for now
}

bool isTextFile(const char* filename) {
  const char* ext = strrchr(filename, '.');
  if (!ext) return false;
  return (strcasecmp(ext, ".txt") == 0);
}

// Load BMP file into persistent buffer for display
// Buffer is allocated once (fullscreen size) and reused
bool loadBMPFile(const char* path) {
  if (!ensureSDCard()) return false;
  
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  
  // Read BMP header
  uint8_t header[54];
  if (f.read(header, 54) != 54) { f.close(); return false; }
  
  // Check BMP signature
  if (header[0] != 'B' || header[1] != 'M') { f.close(); return false; }
  
  // Get image dimensions
  int32_t width = *(int32_t*)&header[18];
  int32_t height = *(int32_t*)&header[22];
  uint16_t bpp = *(uint16_t*)&header[28];
  uint32_t dataOffset = *(uint32_t*)&header[10];
  
  // Handle negative height (top-down BMP)
  bool topDown = (height < 0);
  if (topDown) height = -height;
  
  // Only support 24-bit BMP
  if (bpp != 24) { f.close(); return false; }
  
  // Store original dimensions for scaling
  int origWidth = width;
  int origHeight = height;
  
  // Calculate scale factor to fit screen while maintaining aspect ratio
  // Scale up small images, scale down large images
  float scaleX = (float)SCREEN_WIDTH / origWidth;
  float scaleY = (float)SCREEN_HEIGHT / origHeight;
  float scale = (scaleX < scaleY) ? scaleX : scaleY;  // Use smaller to fit both dimensions
  
  // Calculate scaled dimensions
  int scaledWidth = (int)(origWidth * scale);
  int scaledHeight = (int)(origHeight * scale);
  
  // Center the scaled image on screen
  int offsetX = (SCREEN_WIDTH - scaledWidth) / 2;
  int offsetY = (SCREEN_HEIGHT - scaledHeight) / 2;
  
  bmpViewWidth = SCREEN_WIDTH;   // Buffer is fullscreen
  bmpViewHeight = SCREEN_HEIGHT;
  
  // Allocate buffer if not already allocated (persistent, fullscreen size)
  // Uses PSRAM if available (ESP32-S3), else internal SRAM (ESP32-WROOM-32)
  if (!bmpViewBuffer) {
    bmpViewBuffer = (uint16_t*)psram_malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    if (!bmpViewBuffer) {
      f.close();
      return false;
    }
  }
  
  // Clear buffer (black background)
  memset(bmpViewBuffer, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
  
  // Seek to pixel data
  f.seek(dataOffset);
  
  // Read entire image into temporary buffer first, then scale
  int rowPadding = (4 - (origWidth * 3) % 4) % 4;
  
  // Allocate temporary buffer for original image (RGB565)
  uint16_t* tempImg = (uint16_t*)psram_malloc(origWidth * origHeight * sizeof(uint16_t));
  if (!tempImg) {
    // Fallback: read and scale row by row (slower but uses less memory)
    uint8_t rowData[720 * 3 + 4];  // Max row size
    for (int y = 0; y < origHeight; y++) {
      int srcY = topDown ? y : (origHeight - 1 - y);
      f.read(rowData, origWidth * 3 + rowPadding);
      
      // For each destination row that maps to this source row
      int destYStart = offsetY + (int)(srcY * scale);
      int destYEnd = offsetY + (int)((srcY + 1) * scale);
      if (destYEnd > destYStart) destYEnd = destYStart + 1;  // At least process once
      
      for (int destY = destYStart; destY < destYEnd && destY < SCREEN_HEIGHT; destY++) {
        if (destY < 0) continue;
        for (int destX = offsetX; destX < offsetX + scaledWidth && destX < SCREEN_WIDTH; destX++) {
          if (destX < 0) continue;
          int srcX = (int)((destX - offsetX) / scale);
          if (srcX >= origWidth) srcX = origWidth - 1;
          uint8_t b = rowData[srcX * 3];
          uint8_t g = rowData[srcX * 3 + 1];
          uint8_t r = rowData[srcX * 3 + 2];
          uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
          bmpViewBuffer[destY * SCREEN_WIDTH + destX] = color;
        }
      }
      if (y % 10 == 0) yield();
    }
  } else {
    // Fast path: load entire image then scale
    uint8_t rowData[720 * 3 + 4];
    for (int y = 0; y < origHeight; y++) {
      int destY = topDown ? y : (origHeight - 1 - y);
      f.read(rowData, origWidth * 3 + rowPadding);
      for (int x = 0; x < origWidth; x++) {
        uint8_t b = rowData[x * 3];
        uint8_t g = rowData[x * 3 + 1];
        uint8_t r = rowData[x * 3 + 2];
        uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        tempImg[destY * origWidth + x] = color;
      }
      if (y % 20 == 0) yield();
    }
    
    // Scale image using nearest-neighbor interpolation
    for (int destY = 0; destY < SCREEN_HEIGHT; destY++) {
      int imgY = destY - offsetY;
      if (imgY < 0 || imgY >= scaledHeight) continue;
      int srcY = (int)(imgY / scale);
      if (srcY >= origHeight) srcY = origHeight - 1;
      
      for (int destX = 0; destX < SCREEN_WIDTH; destX++) {
        int imgX = destX - offsetX;
        if (imgX < 0 || imgX >= scaledWidth) continue;
        int srcX = (int)(imgX / scale);
        if (srcX >= origWidth) srcX = origWidth - 1;
        
        bmpViewBuffer[destY * SCREEN_WIDTH + destX] = tempImg[srcY * origWidth + srcX];
      }
      if (destY % 20 == 0) yield();
    }
    free(tempImg);
  }
  
  f.close();
  
  // CRITICAL: Restore TFT state after SD operations
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(100);
  ensureTFTState();
  
  return true;
}

void openFileInViewer(const char* filename) {
  char fullPath[MAX_PATH_LEN];
  if (strlen(currentPath) == 1) {
    snprintf(fullPath, MAX_PATH_LEN, "/%s", filename);
  } else {
    snprintf(fullPath, MAX_PATH_LEN, "%s/%s", currentPath, filename);
  }
  
  if (isTextFile(filename)) {
    // Load text file into Editor
    File f = SD.open(fullPath, FILE_READ);
    if (f) {
      int i = 0;
      while (f.available() && i < NOTE_LEN) {
        char c = f.read();
        if (c >= 32 && c < 127) {  // Printable ASCII only
          editorText[i++] = c;
        }
      }
      editorText[i] = '\0';
      editorCursor = i;  // Cursor at end of text (farthest right)
      f.close();
      // CRITICAL: Restore TFT state after SD operations
      digitalWrite(SD_CS, HIGH);
      delayMicroseconds(100);
      ensureTFTState();
      // Remember which file we opened (for saving back)
      strncpy(editorFilePath, fullPath, MAX_PATH_LEN - 1);
      editorFilePath[MAX_PATH_LEN - 1] = '\0';
      // Open Editor window
      deskOpenWindow(7, "Editor");
    }
  } else if (isImageFile(filename)) {
    // Load BMP file into viewer
    strcpy(bmpViewPath, fullPath);
    if (loadBMPFile(fullPath)) {
      viewingBMP = true;
      // Reset overlay state for new image
      imgViewerOverlayVisible = true;
      imgViewerOverlayTime = millis();
      imgViewerLastCursorX = mouseX;
      imgViewerLastCursorY = mouseY;
      deskOpenWindow(12, "Image");  // App type 12 = Image viewer
    } else {
      showNotification("Can't open BMP");
    }
  } else {
    showNotification("Unknown type");
  }
}

void deleteSelectedFile() {
  if (!sdCardMounted || fileCount == 0 || fileSelected >= fileCount) return;
  
  // Don't delete directories (for safety)
  if (fileIsDir[fileSelected]) {
    showNotification("Can't del dir");
    return;
  }
  
  // Build full path
  char fullPath[MAX_PATH_LEN];
  if (strlen(currentPath) == 1) {
    snprintf(fullPath, MAX_PATH_LEN, "/%s", fileNames[fileSelected]);
  } else {
    snprintf(fullPath, MAX_PATH_LEN, "%s/%s", currentPath, fileNames[fileSelected]);
  }
  
  if (SD.remove(fullPath)) {
    showNotification("Deleted!");
    scanDirectory(currentPath);
    if (fileSelected >= fileCount && fileCount > 0) {
      fileSelected = fileCount - 1;
    }
  } else {
    showNotification("Delete fail");
  }
}

void startRenameMode() {
  if (!sdCardMounted || fileCount == 0 || fileSelected >= fileCount) return;
  if (fileIsDir[fileSelected]) {
    showNotification("Can't rename dir");
    return;
  }
  
  // Split filename into name and extension
  const char* filename = fileNames[fileSelected];
  const char* dot = strrchr(filename, '.');
  if (dot && dot != filename) {
    // Has extension - copy only the name part
    int nameLen = dot - filename;
    if (nameLen > NOTE_LEN) nameLen = NOTE_LEN;
    strncpy(editorText, filename, nameLen);
    editorText[nameLen] = '\0';
    // Store extension (including the dot)
    strncpy(renameExtension, dot, 15);
    renameExtension[15] = '\0';
  } else {
    // No extension
    strncpy(editorText, filename, NOTE_LEN);
    editorText[NOTE_LEN] = '\0';
    renameExtension[0] = '\0';
  }
  
  renameMode = true;
  editorCursor = strlen(editorText);
  editorFilePath[0] = '\0';  // Clear file path for rename mode
  // Open Editor window for keyboard input
  deskOpenWindow(7, "Rename");
}

void executeRename() {
  if (!renameMode || strlen(editorText) == 0) {
    cancelRename();
    return;
  }
  
  // Build new filename with original extension
  char newFilename[MAX_FILENAME_LEN];
  snprintf(newFilename, MAX_FILENAME_LEN, "%s%s", editorText, renameExtension);
  
  // Build old and new paths
  char oldPath[MAX_PATH_LEN];
  char newPath[MAX_PATH_LEN];
  if (strlen(currentPath) == 1) {
    snprintf(oldPath, MAX_PATH_LEN, "/%s", fileNames[fileSelected]);
    snprintf(newPath, MAX_PATH_LEN, "/%s", newFilename);
  } else {
    snprintf(oldPath, MAX_PATH_LEN, "%s/%s", currentPath, fileNames[fileSelected]);
    snprintf(newPath, MAX_PATH_LEN, "%s/%s", currentPath, newFilename);
  }
  
  if (SD.rename(oldPath, newPath)) {
    showNotification("Renamed!");
    scanDirectory(currentPath);
  } else {
    showNotification("Rename fail");
  }
  editorText[0] = '\0';
  editorCursor = 0;
  renameMode = false;
}

void cancelRename() {
  editorText[0] = '\0';
  editorCursor = 0;
  renameMode = false;
}

void scanWiFiNetworks() {
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  g->setTextSize(2);
  g->setTextColor(COLOR_ACCENT);
  g->setCursor(50, 60);
  g->print("Scanning...");
  flushBuffer();
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  int n = WiFi.scanNetworks();
  wifiScanCount = min(n, MAX_WIFI_NETWORKS);
  
  for (int i = 0; i < wifiScanCount; i++) {
    strncpy(wifiNetworks[i], WiFi.SSID(i).c_str(), 32);
    wifiNetworks[i][32] = '\0';
  }
  WiFi.scanDelete();  // CRITICAL: Free scan results memory to prevent leak
  
  wifiScanSel = 0;
  wifiScanMode = true;
  lastButton = 0;
  drawWiFiEditor();  // Draw the network list immediately
}

void drawWiFiEditor() {
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  if (wifiScanMode) {
    // === SSID SELECTION MODE ===
    if (lastButton == 1 && wifiScanSel > 0) wifiScanSel--;
    if (lastButton == 2 && wifiScanSel < wifiScanCount - 1) wifiScanSel++;
    
    // MID - select this network
    if (lastButton == 5 && wifiScanCount > 0) {
      strcpy(wifiSSID, wifiNetworks[wifiScanSel]);
      wifiPass[0] = '\0';
      wifiCursor = 0;
      wifiCharIdx = 0;
      wifiScanMode = false;  // Switch to password entry
    }
    
    // Draw network list
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    
    g->setTextSize(2);
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(40, MENUBAR_HEIGHT + 2);
    g->print("Select WiFi");
    
    int y = MENUBAR_HEIGHT + 24;
    int startIdx = (wifiScanSel > 2) ? wifiScanSel - 2 : 0;
    
    if (wifiScanCount == 0) {
      g->setTextColor(0x6B6D);
      g->setCursor(30, y + 20);
      g->print("No networks");
    } else {
      for (int i = startIdx; i < wifiScanCount && i < startIdx + 4; i++) {
        bool sel = (i == wifiScanSel);
        if (sel) g->fillRect(0, y, SCREEN_WIDTH, 22, COLOR_ACCENT);
        g->setTextColor(sel ? COLOR_BG : COLOR_TEXT);
        g->setCursor(5, y + 3);
        // Truncate long names
        char dispName[15];
        strncpy(dispName, wifiNetworks[i], 14);
        dispName[14] = '\0';
        g->print(dispName);
        y += 24;
      }
    }
    
    g->setTextSize(1);
    g->setTextColor(0x6B6D);
    g->setCursor(5, SCREEN_HEIGHT - 12);
    g->print("MID:Select  SET:Rescan  RST:Back");
    
  } else {
    // === PASSWORD ENTRY MODE ===
    char* currentField = wifiPass;
    int maxLen = 64;
    int len = strlen(currentField);
    
    // UP/DOWN - cycle character
    if (lastButton == 1) {
      wifiCharIdx = (wifiCharIdx + 1) % WIFI_NUM_CHARS;
      if (wifiCursor < maxLen) {
        currentField[wifiCursor] = wifiChars[wifiCharIdx];
        if (wifiCursor >= len) currentField[wifiCursor + 1] = '\0';
      }
    }
    if (lastButton == 2) {
      wifiCharIdx = (wifiCharIdx - 1 + WIFI_NUM_CHARS) % WIFI_NUM_CHARS;
      if (wifiCursor < maxLen) {
        currentField[wifiCursor] = wifiChars[wifiCharIdx];
        if (wifiCursor >= len) currentField[wifiCursor + 1] = '\0';
      }
    }
    
    // LEFT/RIGHT - move cursor
    if (lastButton == 3 && wifiCursor > 0) wifiCursor--;
    if (lastButton == 4 && wifiCursor < len) wifiCursor++;
    
    // MID - delete character
    if (lastButton == 5 && wifiCursor > 0) {
      int slen = strlen(currentField);
      for (int i = wifiCursor; i < slen; i++) {
        currentField[i] = currentField[i + 1];
      }
      wifiCursor--;
    }
    
    // Draw screen
    g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
    
    int y = MENUBAR_HEIGHT + 2;
    
    // Show selected SSID
    g->setTextSize(1);
    g->setTextColor(0x6B6D);
    g->setCursor(5, y);
    g->print("Network: ");
    g->setTextColor(COLOR_ACCENT);
    g->print(wifiSSID);
    y += 14;
    
    // Password label
    g->setTextSize(2);
    g->setTextColor(COLOR_TEXT);
    g->setCursor(5, y);
    g->print("Password:");
    y += 20;
    
    // Password field with cursor
    int fieldLen = strlen(currentField);
    int startChar = (wifiCursor > 10) ? wifiCursor - 10 : 0;
    for (int i = 0; i < 12 && (startChar + i) <= fieldLen; i++) {
      int idx = startChar + i;
      int cx = 5 + i * 18;
      bool isCursor = (idx == wifiCursor);
      if (isCursor) g->fillRect(cx, y, 16, 18, COLOR_ACCENT);
      g->setTextColor(isCursor ? COLOR_BG : COLOR_TEXT);
      g->setCursor(cx + 2, y + 1);
      if (idx < fieldLen) g->print(currentField[idx]);
    }
    y += 24;
    
    // Character selector
    g->setTextSize(2);
    g->setTextColor(COLOR_TEXT);
    g->setCursor(5, y);
    g->print("Char:");
    g->setTextSize(3);
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(75, y - 4);
    g->print(wifiChars[wifiCharIdx]);
    
    g->setTextSize(1);
    g->setTextColor(0x6B6D);
    g->setCursor(5, SCREEN_HEIGHT - 12);
    g->print("SET:Back  RST:Connect");
  }
  
  // Flush buffer to screen
  flushBuffer();
}

// ===== SNAKE GAME FUNCTIONS =====
void snakeInitGame() {
  snakeLen = 3;
  snakeX[0] = SNAKE_COLS / 2;
  snakeY[0] = SNAKE_ROWS / 2;
  snakeX[1] = snakeX[0] - 1;
  snakeY[1] = snakeY[0];
  snakeX[2] = snakeX[0] - 2;
  snakeY[2] = snakeY[0];
  snakeDir = 1;  // Start moving right
  snakeScore = 0;
  snakeGameOver = false;
  snakePlaying = false;
  snakeShowMenu = true;
  snakePaused = false;
  snakeMenuSel = 0;
  snakeMoveInterval = 200;
  snakeLastMove = millis();
  snakeSpawnFood();
}

void snakeSpawnFood() {
  bool valid;
  do {
    valid = true;
    snakeFoodX = random(SNAKE_COLS);
    snakeFoodY = random(SNAKE_ROWS);
    for (int i = 0; i < snakeLen; i++) {
      if (snakeX[i] == snakeFoodX && snakeY[i] == snakeFoodY) {
        valid = false;
        break;
      }
    }
  } while (!valid);
}

void snakeUpdate() {
  if (!snakePlaying || snakeGameOver || snakePaused) return;
  
  unsigned long now = millis();
  if (now - snakeLastMove < (unsigned long)snakeMoveInterval) return;
  snakeLastMove = now;
  
  // Calculate new head position
  int newX = snakeX[0], newY = snakeY[0];
  switch (snakeDir) {
    case 0: newY--; break;  // Up
    case 1: newX++; break;  // Right
    case 2: newY++; break;  // Down
    case 3: newX--; break;  // Left
  }
  
  // Check wall collision
  if (newX < 0 || newX >= SNAKE_COLS || newY < 0 || newY >= SNAKE_ROWS) {
    snakeGameOver = true;
    snakePlaying = false;
    if (snakeScore > snakeHighScore) {
      snakeHighScore = snakeScore;
      saveOSData();
    }
    return;
  }
  
  // Check self collision
  for (int i = 0; i < snakeLen; i++) {
    if (snakeX[i] == newX && snakeY[i] == newY) {
      snakeGameOver = true;
      snakePlaying = false;
      if (snakeScore > snakeHighScore) {
        snakeHighScore = snakeScore;
        saveOSData();
      }
      return;
    }
  }
  
  // Check food
  bool ate = (newX == snakeFoodX && newY == snakeFoodY);
  
  // Move snake
  if (!ate) {
    // Shift body
    for (int i = snakeLen - 1; i > 0; i--) {
      snakeX[i] = snakeX[i - 1];
      snakeY[i] = snakeY[i - 1];
    }
  } else {
    // Grow snake
    for (int i = snakeLen; i > 0; i--) {
      snakeX[i] = snakeX[i - 1];
      snakeY[i] = snakeY[i - 1];
    }
    snakeLen++;
    snakeScore += 10;
    if (snakeMoveInterval > 80) snakeMoveInterval -= 5;
    snakeSpawnFood();
    // Use quiet tone to avoid interrupting music
    if (soundEnabled) playQuietTone(400, 30);  // Higher pitch for eating
  }
  snakeX[0] = newX;
  snakeY[0] = newY;
}

// ===== PAINT APP FUNCTIONS =====
void paintInitCanvas() {
  for (int y = 0; y < PAINT_HEIGHT; y++) {
    for (int x = 0; x < PAINT_WIDTH; x++) {
      paintCanvas[y][x] = 0x0000;  // Black background
    }
  }
  paintCursorX = PAINT_WIDTH / 2;
  paintCursorY = PAINT_HEIGHT / 2;
  paintColorIdx = 0;
  paintColor = paintPalette[0];
  paintSaved = false;
  paintEraser = false;
  
  // Start new session
  paintSessionNum++;
  paintSessionSaved = false;
  paintSessionFile[0] = '\0';
}

void paintSaveToSD() {
  if (!ensureSDCard()) {
    showNotification("No SD card!");
    return;
  }
  ensureOSDataFolder();
  
  // Use session-based filename - same session overwrites same file
  // Save to root folder so files are visible in Files app
  if (!paintSessionSaved) {
    // First save of this session - find next available filename
    int num = 1;
    while (num < 10000) {  // Support up to 9999 paint sessions
      snprintf(paintSessionFile, 64, "/paint%d.bmp", num);
      if (!SD.exists(paintSessionFile)) break;
      num++;
    }
    paintSessionNum = num;  // Update for reference
    paintSessionSaved = true;
  }
  // Else reuse existing paintSessionFile (overwrites previous save of same session)
  
  // Save as BMP file (24-bit RGB, can be opened in image viewers)
  File f = SD.open(paintSessionFile, FILE_WRITE);
  if (f) {
    // BMP Header (54 bytes total)
    uint32_t fileSize = 54 + PAINT_WIDTH * PAINT_HEIGHT * 3;
    uint32_t dataOffset = 54;
    uint32_t headerSize = 40;
    int32_t width = PAINT_WIDTH;
    int32_t height = -PAINT_HEIGHT;  // Negative = top-down
    uint16_t planes = 1;
    uint16_t bpp = 24;
    uint32_t compression = 0;
    uint32_t imageSize = PAINT_WIDTH * PAINT_HEIGHT * 3;
    
    // BMP signature
    f.write('B'); f.write('M');
    f.write((uint8_t*)&fileSize, 4);
    uint32_t reserved = 0;
    f.write((uint8_t*)&reserved, 4);
    f.write((uint8_t*)&dataOffset, 4);
    
    // DIB header
    f.write((uint8_t*)&headerSize, 4);
    f.write((uint8_t*)&width, 4);
    f.write((uint8_t*)&height, 4);
    f.write((uint8_t*)&planes, 2);
    f.write((uint8_t*)&bpp, 2);
    f.write((uint8_t*)&compression, 4);
    f.write((uint8_t*)&imageSize, 4);
    uint32_t zero = 0;
    f.write((uint8_t*)&zero, 4);  // X pixels per meter
    f.write((uint8_t*)&zero, 4);  // Y pixels per meter
    f.write((uint8_t*)&zero, 4);  // Colors used
    f.write((uint8_t*)&zero, 4);  // Important colors
    
    // Pixel data (RGB565 to RGB888)
    for (int y = 0; y < PAINT_HEIGHT; y++) {
      for (int x = 0; x < PAINT_WIDTH; x++) {
        uint16_t c = paintCanvas[y][x];
        // Convert RGB565 to RGB888
        uint8_t r = ((c >> 11) & 0x1F) << 3;
        uint8_t g = ((c >> 5) & 0x3F) << 2;
        uint8_t b = (c & 0x1F) << 3;
        // BMP stores BGR
        f.write(b);
        f.write(g);
        f.write(r);
      }
      // BMP rows must be padded to 4-byte boundary
      int padding = (4 - (PAINT_WIDTH * 3) % 4) % 4;
      for (int p = 0; p < padding; p++) f.write((uint8_t)0);
    }
    f.close();
    // CRITICAL: Restore TFT state after SD operations
    digitalWrite(SD_CS, HIGH);
    delayMicroseconds(100);
    ensureTFTState();
    paintSaved = true;
    showNotification("Saved!");
  } else {
    showNotification("Save failed!");
  }
}

// ===== NON-DESKTOP MODE APPS =====

// Snake app for non-desktop mode with double-buffering
void drawSnakeApp() {
  static unsigned long lastFrame = 0;
  
  // Allocate buffer lazily when entering game mode
  if (!bufferReady) initDoubleBuffer();
  if (!bufferReady) return;  // Can't run without buffer
  
  // Frame rate limiting (~30fps)
  unsigned long now = millis();
  if (now - lastFrame < 33) {
    yield();
    return;
  }
  lastFrame = now;
  
  // Update Snake music for non-desktop mode
  if (snakePlaying && !snakePaused && !snakeGameOver && soundEnabled) {
    if (!snakeMusicPlaying) {
      snakeMusicPlaying = true;
      snakeMusicNote = 0;
      snakeMusicLastNote = millis();
      if (snakeTheme[0][0] > 0) playQuietTone(snakeTheme[0][0], 15);
    }
    unsigned long musicNow = millis();
    if (musicNow - snakeMusicLastNote >= snakeTheme[snakeMusicNote][1]) {
      snakeMusicNote = (snakeMusicNote + 1) % SNAKE_THEME_LEN;
      snakeMusicLastNote = musicNow;
      playQuietTone(snakeTheme[snakeMusicNote][0], 15);
    }
  } else if (snakeMusicPlaying) {
    ledcWrite(BUZZER_PIN, 0);
    snakeMusicPlaying = false;
  }
  
  // Simple input handling using lastButton from main loop
  bool upPressed = (lastButton == 1);
  bool downPressed = (lastButton == 2);
  bool leftPressed = (lastButton == 3);
  bool rightPressed = (lastButton == 4);
  bool midPressed = (lastButton == 5);
  lastButton = 0;  // Consume the button press
  
  // Handle input
  if (snakeShowMenu && !snakePlaying) {
    if (midPressed) {
      playSelect();
      snakeInitGame();
      snakePlaying = true;
      snakeShowMenu = false;
    }
  } else if (snakeGameOver) {
    if (upPressed && snakeMenuSel > 0) { snakeMenuSel = 0; playTick(); }
    if (downPressed && snakeMenuSel < 1) { snakeMenuSel = 1; playTick(); }
    if (midPressed) {
      if (snakeMenuSel == 0) {
        playSelect();
        snakeInitGame();
        snakePlaying = true;
        snakeShowMenu = false;
      } else {
        playClick();
        snakeShowMenu = true;
        snakeGameOver = false;
      }
    }
  } else if (snakePaused) {
    if (upPressed && snakeMenuSel > 0) { snakeMenuSel = 0; playTick(); }
    if (downPressed && snakeMenuSel < 1) { snakeMenuSel = 1; playTick(); }
    if (midPressed) {
      if (snakeMenuSel == 0) {
        playClick();
        snakePaused = false;
      } else {
        playClick();
        snakeShowMenu = true;
        snakePlaying = false;
        snakePaused = false;
      }
    }
  } else if (snakePlaying) {
    if (upPressed && snakeDir != 2) snakeDir = 0;
    if (downPressed && snakeDir != 0) snakeDir = 2;
    if (leftPressed && snakeDir != 1) snakeDir = 3;
    if (rightPressed && snakeDir != 3) snakeDir = 1;
    if (midPressed) {
      playClick();
      snakePaused = true;
      snakeMenuSel = 0;
    }
  }
  
  // Update game
  snakeUpdate();
  
  // Draw to buffer
  int cx = 0, cy = MENUBAR_HEIGHT;
  int cw = SCREEN_WIDTH, ch = SCREEN_HEIGHT - MENUBAR_HEIGHT;
  int infoW = 40;
  int availW = cw - infoW - 6;
  int availH = ch - 4;
  int cellSize = min(availW / SNAKE_COLS, availH / SNAKE_ROWS);
  if (cellSize < 2) cellSize = 2;
  int boardW = SNAKE_COLS * cellSize;
  int boardH = SNAKE_ROWS * cellSize;
  int boardX = cx + 2;
  int boardY = cy + (ch - boardH) / 2;
  int infoX = boardX + boardW + 4;
  
  deskBuffer->fillScreen(COLOR_BG);
  deskBuffer->setTextSize(1);  // Use small text for non-desktop mode
  
  if (snakeShowMenu && !snakePlaying) {
    deskBuffer->setTextColor(COLOR_SNAKE);
    deskBuffer->setCursor(cx + cw/2 - 18, cy + 20);
    deskBuffer->print("SNAKE");
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 45);
    deskBuffer->print("High: ");
    deskBuffer->print(snakeHighScore);
    int btnW = 60, btnH = 18;
    int btnX = cx + cw/2 - btnW/2;
    int btnY = cy + 65;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, 0x07E0);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 12, btnY + 5);
    deskBuffer->print("START");
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(cx + 10, cy + 95);
    deskBuffer->print("Arrows = Move");
    deskBuffer->setCursor(cx + 10, cy + 107);
    deskBuffer->print("MID = Pause");
  } else if (snakeGameOver) {
    deskBuffer->setTextColor(0xF800);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 20);
    deskBuffer->print("GAME OVER");
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 45);
    deskBuffer->print("Score: ");
    deskBuffer->print(snakeScore);
    int btnW = 60, btnH = 18;
    int btnX = cx + cw/2 - btnW/2;
    int btnY = cy + 70;
    uint16_t btnColor = (snakeMenuSel == 0) ? 0x07E0 : 0x4208;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 12, btnY + 5);
    deskBuffer->print("RETRY");
    btnY = cy + 95;
    btnColor = (snakeMenuSel == 1) ? 0x07E0 : 0x4208;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 15, btnY + 5);
    deskBuffer->print("MENU");
  } else if (snakePlaying) {
    deskBuffer->drawRect(boardX - 1, boardY - 1, boardW + 2, boardH + 2, COLOR_HIGHLIGHT);
    deskBuffer->fillRect(boardX + snakeFoodX * cellSize, boardY + snakeFoodY * cellSize, cellSize, cellSize, 0xF800);
    for (int i = 0; i < snakeLen; i++) {
      uint16_t c = (i == 0) ? 0x07E0 : 0x03E0;
      deskBuffer->fillRect(boardX + snakeX[i] * cellSize, boardY + snakeY[i] * cellSize, cellSize, cellSize, c);
    }
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(infoX, boardY);
    deskBuffer->print("Score");
    deskBuffer->setCursor(infoX, boardY + 12);
    deskBuffer->print(snakeScore);
    if (snakePaused) {
      deskBuffer->fillRect(cx + cw/2 - 40, cy + ch/2 - 30, 80, 60, 0x0000);
      deskBuffer->drawRect(cx + cw/2 - 40, cy + ch/2 - 30, 80, 60, COLOR_ACCENT);
      deskBuffer->setTextColor(COLOR_ACCENT);
      deskBuffer->setCursor(cx + cw/2 - 20, cy + ch/2 - 20);
      deskBuffer->print("PAUSED");
      int btnW = 70, btnH = 18;
      int btnX = cx + cw/2 - btnW/2;
      int btnY = cy + ch/2 - 5;
      uint16_t btnColor = (snakeMenuSel == 0) ? 0x07E0 : 0x4208;
      deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
      deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
      deskBuffer->setTextColor(0xFFFF);
      deskBuffer->setCursor(btnX + 14, btnY + 5);
      deskBuffer->print("RESUME");
      btnY = cy + ch/2 + 18;
      btnColor = (snakeMenuSel == 1) ? 0x07E0 : 0x4208;
      deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
      deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
      deskBuffer->setTextColor(0xFFFF);
      deskBuffer->setCursor(btnX + 20, btnY + 5);
      deskBuffer->print("MENU");
    }
  }
  
  // Push buffer to display
  flushBufferRegion(MENUBAR_HEIGHT, SCREEN_HEIGHT - MENUBAR_HEIGHT);
}

// Restart app - confirms and restarts the device
void drawRestartApp() {
  static bool restartConfirm = false;
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  bool midPressed = (lastButton == 5);
  lastButton = 0;
  
  if (midPressed) {
    if (restartConfirm) {
      // Confirmed - restart now
      if (dataDirty) { saveOSData(); dataDirty = false; }
      ESP.restart();
    } else {
      // First press - show confirmation
      restartConfirm = true;
      playSelect();
    }
  }
  
  // Draw UI
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  
  g->setTextSize(2);
  g->setTextColor(0xF800);  // Red
  g->setCursor(70, MENUBAR_HEIGHT + 20);
  g->print("RESTART");
  
  g->setTextSize(1);
  g->setTextColor(COLOR_TEXT);
  
  if (restartConfirm) {
    g->setCursor(40, MENUBAR_HEIGHT + 50);
    g->print("Press MID again to confirm");
    g->setCursor(50, MENUBAR_HEIGHT + 65);
    g->print("or RST to cancel");
  } else {
    g->setCursor(30, MENUBAR_HEIGHT + 50);
    g->print("Press MID to restart device");
    g->setCursor(45, MENUBAR_HEIGHT + 65);
    g->print("All data will be saved");
  }
  
  // Flush buffer to screen
  flushBuffer();
  
  // Reset confirmation if leaving app
  static int lastSelectedApp = -1;
  if (lastSelectedApp != APP_RESTART) {
    restartConfirm = false;
  }
  lastSelectedApp = selectedApp;
}

// Files app for non-desktop mode (view only)
void drawFilesApp() {
  static bool filesInitialized = false;
  static int filesScroll = 0;
  static int filesSel = 0;
  static bool viewingFile = false;
  static char viewFilePath[MAX_PATH_LEN] = "";
  static char viewFileContent[512] = "";
  static int viewScrollOffset = 0;
  
  // Initialize on first run
  if (!filesInitialized) {
    filesInitialized = true;
    strcpy(currentPath, "/");
    scanDirectory(currentPath);
    filesScroll = 0;
    filesSel = 0;
    viewingFile = false;
  }
  
  // Handle input
  bool upPressed = (lastButton == 1);
  bool downPressed = (lastButton == 2);
  bool leftPressed = (lastButton == 3);
  bool rightPressed = (lastButton == 4);
  bool midPressed = (lastButton == 5);
  lastButton = 0;
  
  int maxVisible = 7;  // Files visible at once (fits in available space)
  
  if (viewingBMP) {
    // Viewing a BMP image
    if (midPressed || leftPressed) {
      // Exit BMP view (keep buffer allocated to avoid fragmentation)
      viewingBMP = false;
      playBack();
    }
  } else if (viewingFile) {
    // Viewing a text file
    if (upPressed) viewScrollOffset = max(0, viewScrollOffset - 1);
    if (downPressed) viewScrollOffset++;
    if (midPressed || leftPressed) {
      // Exit file view
      viewingFile = false;
      playBack();
    }
  } else {
    // File list navigation
    if (upPressed && filesSel > 0) {
      filesSel--;
      playTick();
      if (filesSel < filesScroll) filesScroll = filesSel;
    }
    if (downPressed && filesSel < fileCount - 1) {
      filesSel++;
      playTick();
      if (filesSel >= filesScroll + maxVisible) filesScroll = filesSel - maxVisible + 1;
    }
    if (leftPressed) {
      // Go up one directory
      playClick();
      navigateToDir("..");
      filesSel = 0;
      filesScroll = 0;
    }
    if (midPressed && fileCount > 0) {
      if (fileIsDir[filesSel]) {
        // Enter directory
        playSelect();
        navigateToDir(fileNames[filesSel]);
        filesSel = 0;
        filesScroll = 0;
      } else if (isTextFile(fileNames[filesSel])) {
        // View text file
        playSelect();
        char fullPath[MAX_PATH_LEN];
        if (strlen(currentPath) == 1) {
          snprintf(fullPath, MAX_PATH_LEN, "/%s", fileNames[filesSel]);
        } else {
          snprintf(fullPath, MAX_PATH_LEN, "%s/%s", currentPath, fileNames[filesSel]);
        }
        strcpy(viewFilePath, fullPath);
        // Load file content
        File f = SD.open(fullPath, FILE_READ);
        if (f) {
          int len = f.readBytes(viewFileContent, 511);
          viewFileContent[len] = '\0';
          f.close();
          // CRITICAL: Restore TFT state after SD operations
          digitalWrite(SD_CS, HIGH);
          delayMicroseconds(100);
          ensureTFTState();
          viewingFile = true;
          viewScrollOffset = 0;
        } else {
          showNotification("Can't open");
        }
      } else if (isImageFile(fileNames[filesSel])) {
        // View BMP image
        playSelect();
        char fullPath[MAX_PATH_LEN];
        if (strlen(currentPath) == 1) {
          snprintf(fullPath, MAX_PATH_LEN, "/%s", fileNames[filesSel]);
        } else {
          snprintf(fullPath, MAX_PATH_LEN, "%s/%s", currentPath, fileNames[filesSel]);
        }
        strcpy(bmpViewPath, fullPath);
        // Load BMP file
        if (loadBMPFile(fullPath)) {
          viewingBMP = true;
        } else {
          showNotification("Can't open BMP");
        }
      } else {
        // Unsupported file type
        playError();
        showNotification("Unknown file type");
      }
    }
  }
  
  // Use buffer for rendering
  if (!bufferReady) initDoubleBuffer();
  Adafruit_GFX* g = bufferReady ? (Adafruit_GFX*)deskBuffer : (Adafruit_GFX*)&tft;
  
  // Draw UI
  g->fillRect(0, MENUBAR_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - MENUBAR_HEIGHT, COLOR_BG);
  g->setTextSize(1);  // Use small text for file list
  
  if (viewingBMP) {
    // Draw BMP image viewer
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(2, MENUBAR_HEIGHT + 2);
    const char* fname = strrchr(bmpViewPath, '/');
    if (fname) fname++; else fname = bmpViewPath;
    g->print(fname);
    
    g->drawLine(0, MENUBAR_HEIGHT + 12, SCREEN_WIDTH, MENUBAR_HEIGHT + 12, COLOR_HIGHLIGHT);
    
    if (bmpViewBuffer && bmpViewWidth > 0 && bmpViewHeight > 0) {
      // Draw the BMP image from bmpViewBuffer at native resolution
      int imgX = 0;
      int imgY = MENUBAR_HEIGHT + 16;
      for (int y = 0; y < bmpViewHeight && (imgY + y) < SCREEN_HEIGHT - 12; y++) {
        for (int x = 0; x < bmpViewWidth; x++) {
          uint16_t color = bmpViewBuffer[y * SCREEN_WIDTH + x];
          g->drawPixel(imgX + x, imgY + y, color);
        }
      }
    } else {
      g->setTextColor(0xF800);
      g->setCursor(10, MENUBAR_HEIGHT + 40);
      g->print("Load failed");
    }
    
    // Instructions
    g->setTextColor(0x7BEF);
    g->setCursor(2, SCREEN_HEIGHT - 10);
    g->print("MID: Back");
  } else if (viewingFile) {
    // Draw file viewer
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(2, MENUBAR_HEIGHT + 2);
    // Show filename only
    const char* fname = strrchr(viewFilePath, '/');
    if (fname) fname++; else fname = viewFilePath;
    g->print(fname);
    
    g->drawLine(0, MENUBAR_HEIGHT + 12, SCREEN_WIDTH, MENUBAR_HEIGHT + 12, COLOR_HIGHLIGHT);
    
    // Draw file content
    g->setTextColor(COLOR_TEXT);
    int y = MENUBAR_HEIGHT + 15;
    int lineH = 10;
    int maxLines = (SCREEN_HEIGHT - y - 12) / lineH;
    
    // Count lines and draw visible ones
    int lineNum = 0;
    int charIdx = 0;
    int lineStart = 0;
    while (viewFileContent[charIdx] != '\0') {
      if (viewFileContent[charIdx] == '\n' || charIdx - lineStart > 38) {
        if (lineNum >= viewScrollOffset && lineNum < viewScrollOffset + maxLines) {
          g->setCursor(2, y + (lineNum - viewScrollOffset) * lineH);
          for (int i = lineStart; i < charIdx && i < lineStart + 38; i++) {
            if (viewFileContent[i] != '\n' && viewFileContent[i] != '\r') {
              g->print(viewFileContent[i]);
            }
          }
        }
        lineNum++;
        lineStart = charIdx + 1;
      }
      charIdx++;
    }
    // Last line
    if (charIdx > lineStart && lineNum >= viewScrollOffset && lineNum < viewScrollOffset + maxLines) {
      g->setCursor(2, y + (lineNum - viewScrollOffset) * lineH);
      for (int i = lineStart; i < charIdx && i < lineStart + 38; i++) {
        if (viewFileContent[i] != '\n' && viewFileContent[i] != '\r') {
          g->print(viewFileContent[i]);
        }
      }
    }
    
    // Instructions
    g->setTextColor(0x7BEF);
    g->setCursor(2, SCREEN_HEIGHT - 10);
    g->print("UP/DN:Scroll  MID:Back");
  } else {
    // Draw file list
    // Path bar
    g->setTextColor(COLOR_ACCENT);
    g->setCursor(2, MENUBAR_HEIGHT + 2);
    int pathLen = strlen(currentPath);
    if (pathLen > 38) {
      g->print("...");
      g->print(currentPath + pathLen - 35);
    } else {
      g->print(currentPath);
    }
    
    g->drawLine(0, MENUBAR_HEIGHT + 12, SCREEN_WIDTH, MENUBAR_HEIGHT + 12, COLOR_HIGHLIGHT);
    
    if (!sdCardMounted) {
      g->setTextColor(0xF800);
      g->setCursor(60, MENUBAR_HEIGHT + 50);
      g->print("No SD Card");
    } else if (fileCount == 0) {
      g->setTextColor(0x7BEF);
      g->setCursor(70, MENUBAR_HEIGHT + 50);
      g->print("Empty folder");
    } else {
      // Draw file list
      int y = MENUBAR_HEIGHT + 15;
      int itemH = 13;
      for (int i = 0; i < maxVisible && (i + filesScroll) < fileCount; i++) {
        int idx = i + filesScroll;
        int iy = y + i * itemH;
        bool sel = (idx == filesSel);
        
        if (sel) {
          g->fillRect(0, iy, SCREEN_WIDTH, itemH, COLOR_HIGHLIGHT);
          g->setTextColor(COLOR_BG);
        } else {
          g->setTextColor(COLOR_TEXT);
        }
        
        g->setCursor(2, iy + 2);
        // Folder/file indicator
        if (fileIsDir[idx]) {
          g->setTextColor(sel ? 0xFFE0 : 0xFDA0);  // Yellow for folders
          g->print("[D] ");
        } else {
          g->setTextColor(sel ? COLOR_BG : COLOR_TEXT);
          g->print("    ");
        }
        
        // Filename (truncate if needed)
        g->setTextColor(sel ? COLOR_BG : COLOR_TEXT);
        for (int j = 0; j < 34 && fileNames[idx][j] != '\0'; j++) {
          g->print(fileNames[idx][j]);
        }
      }
      
      // Scrollbar
      if (fileCount > maxVisible) {
        int sbX = SCREEN_WIDTH - 4;
        int sbY = MENUBAR_HEIGHT + 15;
        int sbH = maxVisible * 13;
        g->fillRect(sbX, sbY, 3, sbH, 0x4208);
        int thumbH = max(8, sbH * maxVisible / fileCount);
        int thumbY = sbY + (sbH - thumbH) * filesScroll / max(1, fileCount - maxVisible);
        g->fillRect(sbX, thumbY, 3, thumbH, COLOR_ACCENT);
      }
    }
    
    // Instructions
    g->setTextColor(0x7BEF);
    g->setCursor(2, SCREEN_HEIGHT - 10);
    g->print("UP/DN:Nav LEFT:Back MID:Open");
  }
  
  // Flush buffer to screen
  flushBuffer();
}

// Paint app for non-desktop mode with double-buffering
void drawPaintApp() {
  static int paintMouseX = SCREEN_WIDTH / 2;
  static int paintMouseY = SCREEN_HEIGHT / 2;
  static unsigned long lastFrame = 0;
  static unsigned long lastMove = 0;
  
  // Allocate buffer lazily when entering game mode
  if (!bufferReady) initDoubleBuffer();
  if (!bufferReady) return;  // Can't run without buffer
  
  // Frame rate limiting (~30fps)
  unsigned long now = millis();
  if (now - lastFrame < 33) {
    yield();
    return;
  }
  lastFrame = now;
  
  // Cursor movement with acceleration when holding direction
  // Read raw button states for continuous movement detection
  static unsigned long accelStart = 0;  // When we started holding a direction
  static int lastDir = 0;               // Last direction held
  
  bool upHeld = joyUp;
  bool downHeld = joyDown;
  bool leftHeld = joyLeft;
  bool rightHeld = joyRight;
  
  int currentDir = 0;
  if (upHeld) currentDir = 1;
  else if (downHeld) currentDir = 2;
  else if (leftHeld) currentDir = 3;
  else if (rightHeld) currentDir = 4;
  
  if (currentDir > 0) {
    // Track acceleration - reset if direction changed
    if (currentDir != lastDir) {
      accelStart = now;
      lastDir = currentDir;
    }
    
    // Calculate speed based on hold duration (accelerate from 2 to 12 pixels)
    unsigned long holdTime = now - accelStart;
    int speed = 2;
    if (holdTime > 200) speed = 4;
    if (holdTime > 500) speed = 6;
    if (holdTime > 800) speed = 8;
    if (holdTime > 1200) speed = 10;
    if (holdTime > 1600) speed = 12;
    
    int dx = 0, dy = 0;
    if (currentDir == 3) dx = -speed;  // left
    if (currentDir == 4) dx = speed;   // right
    if (currentDir == 1) dy = -speed;  // up
    if (currentDir == 2) dy = speed;   // down
    paintMouseX = constrain(paintMouseX + dx, 0, SCREEN_WIDTH - 1);
    paintMouseY = constrain(paintMouseY + dy, MENUBAR_HEIGHT, SCREEN_HEIGHT - 1);
  } else {
    // Reset acceleration when no direction held
    lastDir = 0;
  }
  bool clicked = (lastButton == 5);
  bool midHeld = joyMid;  // Check if mid is currently held for continuous drawing
  lastButton = 0;
  
  // Layout
  int sidebarW = 22;
  int canvasAreaW = SCREEN_WIDTH - sidebarW - 4;
  int canvasAreaH = SCREEN_HEIGHT - MENUBAR_HEIGHT - 4;
  int cellSize = min(canvasAreaW / PAINT_WIDTH, canvasAreaH / PAINT_HEIGHT);
  if (cellSize < 2) cellSize = 2;
  int canvasW = PAINT_WIDTH * cellSize;
  int canvasH = PAINT_HEIGHT * cellSize;
  int canvasX = 2;
  int canvasY = MENUBAR_HEIGHT + 2;
  int sbX = SCREEN_WIDTH - sidebarW;
  
  // Handle single clicks for UI buttons
  if (clicked) {
    int lx = paintMouseX, ly = paintMouseY;
    if (lx >= sbX) {
      int slx = lx - sbX;
      if (ly >= canvasY && ly < canvasY + 14 && slx >= 2 && slx < 18) { paintEraser = false; playClick(); }
      else if (ly >= canvasY + 16 && ly < canvasY + 30 && slx >= 2 && slx < 18) { paintEraser = true; playClick(); }
      else if (ly >= canvasY + 34 && ly < canvasY + 34 + 50) {
        int row = (ly - canvasY - 34) / 10, col = (slx - 2) / 9;
        if (col >= 0 && col < 2 && row >= 0 && row < 5) {
          int idx = row * 2 + col;
          if (idx < 10) { paintColorIdx = idx; paintColor = paintPalette[idx]; paintEraser = false; playTick(); }
        }
      }
      else if (ly >= SCREEN_HEIGHT - 16 && slx >= 2 && slx < 18) { paintSaveToSD(); }
    }
  }
  
  // Continuous drawing while holding mid button
  if (midHeld) {
    int lx = paintMouseX, ly = paintMouseY;
    if (lx >= canvasX && lx < canvasX + canvasW && ly >= canvasY && ly < canvasY + canvasH) {
      int px = (lx - canvasX) / cellSize, py = (ly - canvasY) / cellSize;
      if (px >= 0 && px < PAINT_WIDTH && py >= 0 && py < PAINT_HEIGHT) {
        uint16_t newColor = paintEraser ? 0xFFFF : paintColor;
        if (paintCanvas[py][px] != newColor) {
          paintCanvas[py][px] = newColor;
        }
      }
    }
  }
  
  // Draw to buffer
  deskBuffer->fillScreen(COLOR_BG);
  deskBuffer->setTextSize(1);  // Use small text for non-desktop mode
  deskBuffer->drawRect(canvasX - 1, canvasY - 1, canvasW + 2, canvasH + 2, COLOR_TEXT);
  for (int py = 0; py < PAINT_HEIGHT; py++) {
    for (int px = 0; px < PAINT_WIDTH; px++) {
      deskBuffer->fillRect(canvasX + px * cellSize, canvasY + py * cellSize, cellSize, cellSize, paintCanvas[py][px]);
    }
  }
  deskBuffer->fillRect(sbX, MENUBAR_HEIGHT, sidebarW, SCREEN_HEIGHT - MENUBAR_HEIGHT, 0x4208);
  deskBuffer->drawLine(sbX, MENUBAR_HEIGHT, sbX, SCREEN_HEIGHT, COLOR_TEXT);
  uint16_t brushBtnColor = paintEraser ? 0x4208 : 0x07E0;
  deskBuffer->fillRect(sbX + 2, canvasY, 16, 14, brushBtnColor);
  deskBuffer->drawRect(sbX + 2, canvasY, 16, 14, COLOR_TEXT);
  deskBuffer->setTextColor(0x0000);
  deskBuffer->setCursor(sbX + 5, canvasY + 3);
  deskBuffer->print("B");
  uint16_t eraserBtnColor = paintEraser ? 0xF800 : 0x4208;
  deskBuffer->fillRect(sbX + 2, canvasY + 16, 16, 14, eraserBtnColor);
  deskBuffer->drawRect(sbX + 2, canvasY + 16, 16, 14, COLOR_TEXT);
  deskBuffer->setTextColor(0xFFFF);
  deskBuffer->setCursor(sbX + 5, canvasY + 19);
  deskBuffer->print("E");
  for (int i = 0; i < 10; i++) {
    int col = i % 2, row = i / 2;
    int palX = sbX + 2 + col * 9, palY = canvasY + 34 + row * 10;
    deskBuffer->fillRect(palX, palY, 8, 8, paintPalette[i]);
    if (i == paintColorIdx && !paintEraser) deskBuffer->drawRect(palX - 1, palY - 1, 10, 10, COLOR_ACCENT);
  }
  deskBuffer->fillRect(sbX + 2, SCREEN_HEIGHT - 16, 16, 12, 0x001F);
  deskBuffer->drawRect(sbX + 2, SCREEN_HEIGHT - 16, 16, 12, COLOR_TEXT);
  deskBuffer->setTextColor(0xFFFF);
  deskBuffer->setCursor(sbX + 5, SCREEN_HEIGHT - 13);
  deskBuffer->print("S");
  
  // Cursor highlight
  if (paintMouseX >= canvasX && paintMouseX < canvasX + canvasW && paintMouseY >= canvasY && paintMouseY < canvasY + canvasH) {
    int px = (paintMouseX - canvasX) / cellSize, py = (paintMouseY - canvasY) / cellSize;
    if (px >= 0 && px < PAINT_WIDTH && py >= 0 && py < PAINT_HEIGHT) {
      deskBuffer->drawRect(canvasX + px * cellSize, canvasY + py * cellSize, cellSize, cellSize, COLOR_ACCENT);
    }
  }
  
  // Mouse pointer
  deskBuffer->fillTriangle(paintMouseX, paintMouseY, paintMouseX + 8, paintMouseY + 4, paintMouseX + 4, paintMouseY + 8, COLOR_TEXT);
  deskBuffer->drawTriangle(paintMouseX, paintMouseY, paintMouseX + 8, paintMouseY + 4, paintMouseX + 4, paintMouseY + 8, 0x0000);
  
  // Push buffer to display
  flushBufferRegion(MENUBAR_HEIGHT, SCREEN_HEIGHT - MENUBAR_HEIGHT);
}

// Tetris app for non-desktop mode with double-buffering
void drawTetrisApp() {
  static unsigned long lastFrame = 0;
  
  // Allocate buffer lazily when entering game mode
  if (!bufferReady) initDoubleBuffer();
  if (!bufferReady) return;  // Can't run without buffer
  
  // Frame rate limiting (~30fps)
  unsigned long now = millis();
  if (now - lastFrame < 33) {
    yield();
    return;
  }
  lastFrame = now;
  
  // Update Tetris music for non-desktop mode
  if (tetrisPlaying && !tetrisPaused && !tetrisGameOver && soundEnabled) {
    if (!tetrisMusicPlaying) {
      tetrisMusicPlaying = true;
      tetrisMusicNote = 0;
      tetrisMusicLastNote = millis();
      if (tetrisTheme[0][0] > 0) playQuietTone(tetrisTheme[0][0], 15);
    }
    unsigned long musicNow = millis();
    if (musicNow - tetrisMusicLastNote >= tetrisTheme[tetrisMusicNote][1]) {
      tetrisMusicNote = (tetrisMusicNote + 1) % TETRIS_THEME_LEN;
      tetrisMusicLastNote = musicNow;
      playQuietTone(tetrisTheme[tetrisMusicNote][0], 15);
    }
  } else if (tetrisMusicPlaying) {
    ledcWrite(BUZZER_PIN, 0);
    tetrisMusicPlaying = false;
  }
  
  // Simple input handling using lastButton from main loop
  bool upPressed = (lastButton == 1);
  bool downPressed = (lastButton == 2);
  bool leftPressed = (lastButton == 3);
  bool rightPressed = (lastButton == 4);
  bool midPressed = (lastButton == 5);
  lastButton = 0;  // Consume the button press
  
  // Handle input
  if (tetrisShowMenu && !tetrisPlaying) {
    if (midPressed) {
      playSelect();
      tetrisInitGame();
      tetrisPlaying = true;
      tetrisShowMenu = false;
    }
  } else if (tetrisGameOver) {
    if (upPressed && tetrisMenuSel > 0) { tetrisMenuSel = 0; playTick(); }
    if (downPressed && tetrisMenuSel < 1) { tetrisMenuSel = 1; playTick(); }
    if (midPressed) {
      if (tetrisMenuSel == 0) {
        playSelect();
        tetrisInitGame();
        tetrisPlaying = true;
        tetrisShowMenu = false;
      } else {
        playClick();
        tetrisShowMenu = true;
        tetrisGameOver = false;
      }
    }
  } else if (tetrisPaused) {
    if (upPressed && tetrisMenuSel > 0) { tetrisMenuSel = 0; playTick(); }
    if (downPressed && tetrisMenuSel < 1) { tetrisMenuSel = 1; playTick(); }
    if (midPressed) {
      if (tetrisMenuSel == 0) {
        playClick();
        tetrisPaused = false;
      } else {
        playClick();
        tetrisShowMenu = true;
        tetrisPlaying = false;
        tetrisPaused = false;
      }
    }
  } else if (tetrisPlaying) {
    if (leftPressed) tetrisMove(-1);
    if (rightPressed) tetrisMove(1);
    if (upPressed) tetrisRotate();
    if (downPressed) tetrisDrop();
    if (midPressed) {
      playClick();
      tetrisPaused = true;
      tetrisMenuSel = 0;
    }
  }
  
  // Update game
  tetrisUpdate();
  
  // Draw to buffer
  int cx = 0, cy = MENUBAR_HEIGHT;
  int cw = SCREEN_WIDTH, ch = SCREEN_HEIGHT - MENUBAR_HEIGHT;
  int cellSize = 6;
  int boardW = TETRIS_COLS * cellSize;
  int boardH = TETRIS_ROWS * cellSize;
  int boardX = cx + (cw - boardW - 50) / 2;
  int boardY = cy + (ch - boardH) / 2;
  int infoX = boardX + boardW + 8;
  
  deskBuffer->fillScreen(COLOR_BG);
  deskBuffer->setTextSize(1);  // Use small text for non-desktop mode
  
  if (tetrisShowMenu && !tetrisPlaying) {
    deskBuffer->setTextColor(0x07FF);
    deskBuffer->setCursor(cx + cw/2 - 18, cy + 20);
    deskBuffer->print("TETRIS");
    for (int i = 0; i < 7; i++) {
      deskBuffer->fillRect(cx + cw/2 - 42 + i * 12, cy + 35, 10, 10, tetrisColors[i]);
    }
    int btnW = 60, btnH = 18;
    int btnX = cx + cw/2 - btnW/2;
    int btnY = cy + 55;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, 0x07E0);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 12, btnY + 5);
    deskBuffer->print("START");
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(cx + 10, cy + 85);
    deskBuffer->print("L/R = Move piece");
    deskBuffer->setCursor(cx + 10, cy + 97);
    deskBuffer->print("UP  = Rotate");
    deskBuffer->setCursor(cx + 10, cy + 109);
    deskBuffer->print("DN  = Fast drop");
  } else if (tetrisGameOver) {
    deskBuffer->setTextColor(0xF800);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 20);
    deskBuffer->print("GAME OVER");
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 45);
    deskBuffer->print("Score: ");
    deskBuffer->print(tetrisScore);
    deskBuffer->setCursor(cx + cw/2 - 30, cy + 60);
    deskBuffer->print("Lines: ");
    deskBuffer->print(tetrisLines);
    int btnW = 60, btnH = 18;
    int btnX = cx + cw/2 - btnW/2;
    int btnY = cy + 80;
    uint16_t btnColor = (tetrisMenuSel == 0) ? 0x07E0 : 0x4208;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 12, btnY + 5);
    deskBuffer->print("RETRY");
    btnY = cy + 105;
    btnColor = (tetrisMenuSel == 1) ? 0x07E0 : 0x4208;
    deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
    deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
    deskBuffer->setTextColor(0xFFFF);
    deskBuffer->setCursor(btnX + 15, btnY + 5);
    deskBuffer->print("MENU");
  } else if (tetrisPlaying) {
    deskBuffer->drawRect(boardX - 1, boardY - 1, boardW + 2, boardH + 2, COLOR_HIGHLIGHT);
    for (int r = 0; r < TETRIS_ROWS; r++) {
      for (int c = 0; c < TETRIS_COLS; c++) {
        if (tetrisBoard[r][c] > 0) {
          deskBuffer->fillRect(boardX + c * cellSize, boardY + r * cellSize, cellSize - 1, cellSize - 1, tetrisColors[tetrisBoard[r][c] - 1]);
        }
      }
    }
    uint16_t piece = tetrisPieces[tetrisPieceType][tetrisPieceRot];
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        if (piece & (0x8000 >> (r * 4 + c))) {
          int br = tetrisPieceY + r;
          int bc = tetrisPieceX + c;
          if (br >= 0 && br < TETRIS_ROWS && bc >= 0 && bc < TETRIS_COLS) {
            deskBuffer->fillRect(boardX + bc * cellSize, boardY + br * cellSize, cellSize - 1, cellSize - 1, tetrisColors[tetrisPieceType]);
          }
        }
      }
    }
    deskBuffer->setTextColor(COLOR_TEXT);
    deskBuffer->setCursor(infoX, boardY);
    deskBuffer->print("Score");
    deskBuffer->setCursor(infoX, boardY + 10);
    deskBuffer->print(tetrisScore);
    deskBuffer->setCursor(infoX, boardY + 28);
    deskBuffer->print("Lines");
    deskBuffer->setCursor(infoX, boardY + 38);
    deskBuffer->print(tetrisLines);
    if (tetrisPaused) {
      deskBuffer->fillRect(cx + cw/2 - 50, cy + ch/2 - 40, 100, 80, 0x0000);
      deskBuffer->drawRect(cx + cw/2 - 50, cy + ch/2 - 40, 100, 80, 0xFFFF);
      deskBuffer->setTextColor(0xFFE0);
      deskBuffer->setCursor(cx + cw/2 - 21, cy + ch/2 - 30);
      deskBuffer->print("PAUSED");
      int btnW = 70, btnH = 18;
      int btnX = cx + cw/2 - btnW/2;
      int btnY = cy + ch/2 - 10;
      uint16_t btnColor = (tetrisMenuSel == 0) ? 0x07E0 : 0x4208;
      deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
      deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
      deskBuffer->setTextColor(0xFFFF);
      deskBuffer->setCursor(btnX + 14, btnY + 5);
      deskBuffer->print("RESUME");
      btnY = cy + ch/2 + 15;
      btnColor = (tetrisMenuSel == 1) ? 0x07E0 : 0x4208;
      deskBuffer->fillRect(btnX, btnY, btnW, btnH, btnColor);
      deskBuffer->drawRect(btnX, btnY, btnW, btnH, 0xFFFF);
      deskBuffer->setTextColor(0xFFFF);
      deskBuffer->setCursor(btnX + 20, btnY + 5);
      deskBuffer->print("MENU");
    }
  }
  
  // Push buffer to display
  flushBufferRegion(MENUBAR_HEIGHT, SCREEN_HEIGHT - MENUBAR_HEIGHT);
}