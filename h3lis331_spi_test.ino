/*
  h3lis331_spi_test.ino

  Bench test: reads BOTH H3LIS331 high-g accelerometers over SPI (4-wire)
  and streams X/Y/Z readings to the USB serial console.
  Target: RP2350 (e.g. Pico 2) via the arduino-pico core.

  SAFETY: this sketch never starts DShot. The two ESC signal pins are held
  LOW as plain GPIO outputs, so the ESCs never receive a valid frame and
  cannot arm - the robot will not move. LEDs and the CRSF receiver UART
  are left untouched.

  No extra libraries needed - talks to the sensor registers directly with
  the core's built-in SPI.h, so the WHO_AM_I and config read-backs can be
  printed for debugging.

  Wiring (from Meltybrain BOM - wiring.csv). H3LIS331 breakout labels are
  the I2C names; in SPI mode SCL = clock, SDA = MOSI, SDO = MISO.

    ACCEL, LEFT  -> hardware SPI1 (arduino-pico object: SPI1)
      CS   -> GP13 (phys 17)
      SDO  -> GP12 (phys 16)  SPI1 RX / MISO
      SCL  -> GP10 (phys 14)  SPI1 SCK
      SDA  -> GP11 (phys 15)  SPI1 TX / MOSI
              (CSV says "15 / GP1" - physical pin 15 is GP11, and GP11 is
               the SPI1 TX pin, so GP1 is assumed to be a typo)

    ACCEL, RIGHT -> hardware SPI0 (arduino-pico object: SPI)
      CS   -> GP5  (phys 7)
      SDO  -> GP4  (phys 6)   SPI0 RX / MISO
      SCL  -> GP2  (phys 4)   SPI0 SCK
      SDA  -> GP3  (phys 5)   SPI0 TX / MOSI

    The two sensors are on separate SPI buses.

    Both: VIN -> 3V3(OUT), GND -> GND. 3.3V part, no level shifting.

  Sanity check: sitting still, one axis of each sensor should read about
  +/-1 g and the other two about 0 g. "|a|" (magnitude) should be ~1 g.
  At +/-100 g range one count is ~0.049 g, so expect a little jitter.

  Output lines use "name:value" pairs, so the Arduino Serial Plotter can
  graph them directly as well as being readable in the Serial Monitor.
*/

#include <SPI.h>

// ---- Configuration -------------------------------------------------------

// Left accelerometer (SPI1)
constexpr uint8_t L_CS_PIN   = 13;
constexpr uint8_t L_MISO_PIN = 12;
constexpr uint8_t L_SCK_PIN  = 10;
constexpr uint8_t L_MOSI_PIN = 11;

// Right accelerometer (SPI0)
constexpr uint8_t R_CS_PIN   = 5;
constexpr uint8_t R_MISO_PIN = 4;
constexpr uint8_t R_SCK_PIN  = 2;
constexpr uint8_t R_MOSI_PIN = 3;

// ESC signal pins - held LOW, DShot is never started.
constexpr uint8_t MOTOR_LEFT_PIN  = 22;
constexpr uint8_t MOTOR_RIGHT_PIN = 21;

// H3LIS331 SPI: max 10 MHz, mode 3 (clock idles high, sample on rising edge).
constexpr uint32_t SPI_CLOCK_HZ = 1000000;

// Range: 100, 200 or 400 (g). 100 g has the finest resolution for a bench
// test; switch to 400 once it's mounted and actually spinning.
constexpr uint16_t RANGE_G = 100;

// Output data rate: 50, 100, 400 or 1000 (Hz).
constexpr uint16_t DATA_RATE_HZ = 400;

constexpr uint32_t PRINT_INTERVAL_MS = 100;  // 10 Hz serial print rate
constexpr uint32_t RETRY_INTERVAL_MS = 2000; // re-probe a missing sensor

// ---- H3LIS331 registers --------------------------------------------------

constexpr uint8_t REG_WHO_AM_I  = 0x0F;
constexpr uint8_t REG_CTRL1     = 0x20;  // power mode, data rate, axis enable
constexpr uint8_t REG_CTRL4     = 0x23;  // BDU, endianness, full scale, SPI mode
constexpr uint8_t REG_OUT_X_L   = 0x28;  // X_L, X_H, Y_L, Y_H, Z_L, Z_H follow
constexpr uint8_t WHO_AM_I_VAL  = 0x32;

constexpr uint8_t SPI_READ      = 0x80;  // bit 7: 1 = read
constexpr uint8_t SPI_AUTOINC   = 0x40;  // bit 6: 1 = auto-increment address

// ---- Sensor bookkeeping --------------------------------------------------

struct Accel {
  const char *name;
  arduino::HardwareSPI *bus;
  uint8_t cs;
  bool ok;
  uint32_t lastTryMs;
};

Accel accelL = { "L", &SPI1, L_CS_PIN, false, 0 };
Accel accelR = { "R", &SPI,  R_CS_PIN, false, 0 };

const SPISettings accelSpi(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3);

float gPerCount;  // set from RANGE_G in setup()

// ---- Low-level SPI register access ---------------------------------------

void writeReg(Accel &a, uint8_t reg, uint8_t value) {
  a.bus->beginTransaction(accelSpi);
  digitalWrite(a.cs, LOW);
  a.bus->transfer(reg & 0x3F);  // bit 7 = 0 (write), bit 6 = 0 (single)
  a.bus->transfer(value);
  digitalWrite(a.cs, HIGH);
  a.bus->endTransaction();
}

void readRegs(Accel &a, uint8_t reg, uint8_t *buf, size_t len) {
  uint8_t cmd = (reg & 0x3F) | SPI_READ;
  if (len > 1) cmd |= SPI_AUTOINC;
  a.bus->beginTransaction(accelSpi);
  digitalWrite(a.cs, LOW);
  a.bus->transfer(cmd);
  for (size_t i = 0; i < len; i++) buf[i] = a.bus->transfer(0x00);
  digitalWrite(a.cs, HIGH);
  a.bus->endTransaction();
}

uint8_t readReg(Accel &a, uint8_t reg) {
  uint8_t v;
  readRegs(a, reg, &v, 1);
  return v;
}

// ---- Sensor setup ----------------------------------------------------------

uint8_t ctrl1Value() {
  uint8_t dr;
  switch (DATA_RATE_HZ) {
    case 50:   dr = 0b00; break;
    case 100:  dr = 0b01; break;
    case 1000: dr = 0b11; break;
    default:   dr = 0b10; break;  // 400 Hz
  }
  // PM = 001 (normal mode), DR, Zen | Yen | Xen
  return (0b001 << 5) | (dr << 3) | 0b111;
}

uint8_t ctrl4Value() {
  uint8_t fs;
  switch (RANGE_G) {
    case 200: fs = 0b01; break;
    case 400: fs = 0b11; break;
    default:  fs = 0b00; break;  // 100 g
  }
  // BDU = 1 (don't mix high/low bytes from different samples),
  // BLE = 0 (little endian), SIM = 0 (4-wire SPI)
  return 0x80 | (fs << 4);
}

bool initAccel(Accel &a) {
  a.lastTryMs = millis();

  uint8_t id = readReg(a, REG_WHO_AM_I);
  Serial.print("[");  Serial.print(a.name);
  Serial.print("] WHO_AM_I = 0x"); Serial.print(id, HEX);
  if (id != WHO_AM_I_VAL) {
    Serial.print(" (expected 0x32) - ");
    if (id == 0x00 || id == 0xFF)
      Serial.println("no response: check CS/SCK/MISO wiring and power");
    else
      Serial.println("unexpected value: check MOSI wiring and SPI mode");
    return false;
  }
  Serial.println(" OK");

  writeReg(a, REG_CTRL4, ctrl4Value());
  writeReg(a, REG_CTRL1, ctrl1Value());

  // Read back the config. A mismatch here with a good WHO_AM_I usually
  // means MOSI (breakout "SDA") isn't connected - reads work, writes don't.
  uint8_t c1 = readReg(a, REG_CTRL1);
  uint8_t c4 = readReg(a, REG_CTRL4);
  bool ok = (c1 == ctrl1Value()) && (c4 == ctrl4Value());
  Serial.print("[");  Serial.print(a.name);
  Serial.print("] CTRL1 = 0x"); Serial.print(c1, HEX);
  Serial.print(", CTRL4 = 0x"); Serial.print(c4, HEX);
  Serial.println(ok ? " OK" : " MISMATCH - check MOSI (SDA) wiring");
  return ok;
}

// Reads X/Y/Z in g. Data is 12-bit, left-justified in 16-bit two's complement.
void readAccel(Accel &a, float &x, float &y, float &z) {
  uint8_t b[6];
  readRegs(a, REG_OUT_X_L, b, 6);
  int16_t rx = (int16_t)(b[0] | (b[1] << 8)) >> 4;
  int16_t ry = (int16_t)(b[2] | (b[3] << 8)) >> 4;
  int16_t rz = (int16_t)(b[4] | (b[5] << 8)) >> 4;
  x = rx * gPerCount;
  y = ry * gPerCount;
  z = rz * gPerCount;
}

void printAccel(Accel &a) {
  if (!a.ok) {
    Serial.print(a.name); Serial.print(":MISSING");
    return;
  }
  float x, y, z;
  readAccel(a, x, y, z);
  float mag = sqrtf(x * x + y * y + z * z);
  Serial.print(a.name); Serial.print("x:"); Serial.print(x, 3);
  Serial.print(' ');
  Serial.print(a.name); Serial.print("y:"); Serial.print(y, 3);
  Serial.print(' ');
  Serial.print(a.name); Serial.print("z:"); Serial.print(z, 3);
  Serial.print(' ');
  Serial.print(a.name); Serial.print("|a|:"); Serial.print(mag, 3);
}

// ---- Arduino entry points --------------------------------------------------

void setup() {
  // Keep the ESCs quiet first thing: a steady LOW is never a valid DShot
  // frame, so the ESCs stay disarmed for the whole test.
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  digitalWrite(MOTOR_LEFT_PIN, LOW);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  digitalWrite(MOTOR_RIGHT_PIN, LOW);

  // Chip selects idle HIGH before the buses start.
  pinMode(L_CS_PIN, OUTPUT);
  digitalWrite(L_CS_PIN, HIGH);
  pinMode(R_CS_PIN, OUTPUT);
  digitalWrite(R_CS_PIN, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for the USB serial connection

  Serial.println("H3LIS331 dual SPI test (motors disabled)");

  // Pin assignment must happen before begin(). CS is driven manually
  // as a GPIO so it stays low across multi-byte reads.
  SPI1.setRX(L_MISO_PIN);
  SPI1.setSCK(L_SCK_PIN);
  SPI1.setTX(L_MOSI_PIN);
  SPI1.begin();

  SPI.setRX(R_MISO_PIN);
  SPI.setSCK(R_SCK_PIN);
  SPI.setTX(R_MOSI_PIN);
  SPI.begin();

  // Full-scale span is 2 * RANGE_G over 4096 counts (12-bit).
  // Matches the datasheet: ~49 / 98 / 195 mg per count at 100 / 200 / 400 g.
  gPerCount = (2.0f * RANGE_G) / 4096.0f;

  Serial.print("Range: +/-"); Serial.print(RANGE_G);
  Serial.print(" g, data rate: "); Serial.print(DATA_RATE_HZ);
  Serial.print(" Hz, SPI clock: "); Serial.print(SPI_CLOCK_HZ);
  Serial.println(" Hz");

  delay(10);  // sensor boot time after power-up
  accelL.ok = initAccel(accelL);
  accelR.ok = initAccel(accelR);
  Serial.println();
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  // Re-probe a sensor that didn't come up, so you can fix wiring live.
  if (!accelL.ok && now - accelL.lastTryMs >= RETRY_INTERVAL_MS)
    accelL.ok = initAccel(accelL);
  if (!accelR.ok && now - accelR.lastTryMs >= RETRY_INTERVAL_MS)
    accelR.ok = initAccel(accelR);

  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;

  printAccel(accelL);
  Serial.print("  ");
  printAccel(accelR);
  Serial.println();
}
