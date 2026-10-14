/*
  neopixel_led_test.ino

  Bench test: flashes the top and bottom NeoPixel strips so you can check
  wiring, pixel count, pixel order and color order. Progress is printed to
  the USB serial console.
  Target: RP2350 (e.g. Pico 2) via the arduino-pico core.

  SAFETY: this sketch never starts DShot. The two ESC signal pins are held
  LOW as plain GPIO outputs, so the ESCs never receive a valid frame and
  cannot arm - the robot will not move. Accelerometers and the receiver
  UART are left untouched.

  Library:  Adafruit NeoPixel (Arduino Library Manager)
            https://github.com/adafruit/Adafruit_NeoPixel

  Wiring (from Meltybrain BOM - wiring.csv):
            LED, TOP  DIN -> GP18 (phys 24)
            LED, BOT  DIN -> GP19 (phys 25)
            Strip GND must be shared with the Pico GND.

  Test sequence (repeats forever):
    1. Chase  - one pixel at a time, in order, TOP strip then BOT strip.
                Pixel 0 is the one nearest the DIN wire. If the chase stops
                early or skips pixels, NUM_PIXELS_* is wrong or a pixel's
                data line is broken.
    2. Colors - whole strips go red, then green, then blue. If red shows as
                green (or vice versa), change NEO_GRB to NEO_RGB below.
    3. Flash  - both strips flash white together a few times.
*/

#include <Adafruit_NeoPixel.h>

// ---- Configuration - adjust for your hardware ----------------------------

constexpr uint8_t  LED_TOP_PIN    = 18;  // GP18
constexpr uint8_t  LED_BOT_PIN    = 19;  // GP19

// Set these to the number of pixels actually on each strip/ring.
constexpr uint16_t NUM_PIXELS_TOP = 8;
constexpr uint16_t NUM_PIXELS_BOT = 8;

// Most WS2812B / NeoPixel parts are GRB at 800 kHz. Use NEO_RGB if colors
// come out swapped, or NEO_GRBW for RGBW pixels.
constexpr neoPixelType PIXEL_TYPE = NEO_GRB + NEO_KHZ800;

// 0-255. Kept low so a bench test off USB power doesn't brown out the board
// (a full-white pixel at 255 draws ~60 mA).
constexpr uint8_t  BRIGHTNESS     = 32;

constexpr uint32_t CHASE_STEP_MS  = 150;  // time each pixel stays lit
constexpr uint32_t COLOR_HOLD_MS  = 700;  // time each solid color is shown
constexpr uint32_t FLASH_ON_MS    = 200;
constexpr uint32_t FLASH_OFF_MS   = 200;
constexpr uint8_t  FLASH_COUNT    = 3;

// ESC signal pins - held LOW, DShot is never started.
constexpr uint8_t  MOTOR_LEFT_PIN  = 22;
constexpr uint8_t  MOTOR_RIGHT_PIN = 21;

// --------------------------------------------------------------------------

Adafruit_NeoPixel ledTop(NUM_PIXELS_TOP, LED_TOP_PIN, PIXEL_TYPE);
Adafruit_NeoPixel ledBot(NUM_PIXELS_BOT, LED_BOT_PIN, PIXEL_TYPE);

void allOff() {
  ledTop.clear(); ledTop.show();
  ledBot.clear(); ledBot.show();
}

void fill(Adafruit_NeoPixel &strip, uint32_t color) {
  strip.fill(color);
  strip.show();
}

// Light one pixel at a time, in order, in the given color.
void chase(Adafruit_NeoPixel &strip, const char *name, uint32_t color) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    Serial.print(name); Serial.print(" pixel "); Serial.println(i);
    strip.clear();
    strip.setPixelColor(i, color);
    strip.show();
    delay(CHASE_STEP_MS);
  }
  strip.clear();
  strip.show();
}

void setup() {
  // Keep the ESCs quiet first thing: a steady LOW is never a valid DShot
  // frame, so the ESCs stay disarmed for the whole test.
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  digitalWrite(MOTOR_LEFT_PIN, LOW);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  digitalWrite(MOTOR_RIGHT_PIN, LOW);

  Serial.begin(115200);
  // Wait briefly for the USB console, but run the LEDs even without one.
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  ledTop.begin();
  ledBot.begin();
  ledTop.setBrightness(BRIGHTNESS);
  ledBot.setBrightness(BRIGHTNESS);
  allOff();

  Serial.println("NeoPixel LED test (motors disabled)");
  Serial.print("TOP: GP"); Serial.print(LED_TOP_PIN);
  Serial.print(", "); Serial.print(NUM_PIXELS_TOP); Serial.println(" pixels");
  Serial.print("BOT: GP"); Serial.print(LED_BOT_PIN);
  Serial.print(", "); Serial.print(NUM_PIXELS_BOT); Serial.println(" pixels");
  Serial.println();
}

void loop() {
  const uint32_t RED   = Adafruit_NeoPixel::Color(255, 0, 0);
  const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 255, 0);
  const uint32_t BLUE  = Adafruit_NeoPixel::Color(0, 0, 255);
  const uint32_t WHITE = Adafruit_NeoPixel::Color(255, 255, 255);

  // 1. Chase, in order: TOP strip, then BOT strip.
  Serial.println("-- Chase --");
  chase(ledTop, "TOP", WHITE);
  chase(ledBot, "BOT", WHITE);

  // 2. Solid colors on both strips, to check color order.
  Serial.println("-- Colors --");
  const uint32_t colors[] = { RED, GREEN, BLUE };
  const char *colorNames[] = { "red", "green", "blue" };
  for (uint8_t c = 0; c < 3; c++) {
    Serial.print("Both strips: "); Serial.println(colorNames[c]);
    fill(ledTop, colors[c]);
    fill(ledBot, colors[c]);
    delay(COLOR_HOLD_MS);
  }
  allOff();

  // 3. Flash both strips together.
  Serial.println("-- Flash --");
  for (uint8_t n = 0; n < FLASH_COUNT; n++) {
    fill(ledTop, WHITE);
    fill(ledBot, WHITE);
    delay(FLASH_ON_MS);
    allOff();
    delay(FLASH_OFF_MS);
  }

  Serial.println();
  delay(500);
}
