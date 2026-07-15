/*
  h3lis331_serial_test.ino

  Bench test: reads the Adafruit H3LIS331 high-g accelerometer over I2C
  and streams X/Y/Z readings to USB serial. Target: RP2040 (Raspberry Pi
  Pico) via the arduino-pico core.

  Library:  Adafruit LIS331 (Arduino Library Manager)
            Also installs its dependencies: Adafruit BusIO, Adafruit Unified Sensor
            https://github.com/adafruit/Adafruit_LIS331

  Wiring (I2C):
            H3LIS331 VIN  -> Pico 3V3(OUT)   (this sensor is a 3.3V part -
                                               RP2040 is natively 3.3V logic
                                               too, so NO level shifter is
                                               needed here, unlike on a 5V
                                               AVR Arduino)
            H3LIS331 GND  -> Pico GND
            H3LIS331 SCL  -> I2C_SCL_PIN (default GP5 below)
            H3LIS331 SDA  -> I2C_SDA_PIN (default GP4 below)

  I2C address: 0x18 by default. If you've tied the board's SDO pin to 3V,
  it becomes 0x19 - update ACCEL_I2C_ADDR below to match.
*/

#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

// ---- Configuration - adjust for your wiring ----
constexpr uint8_t I2C_SDA_PIN    = 4;    // GP4
constexpr uint8_t I2C_SCL_PIN    = 5;    // GP5
constexpr uint8_t ACCEL_I2C_ADDR = 0x18; // 0x18 default, 0x19 if SDO tied to 3V

constexpr uint32_t PRINT_INTERVAL_MS = 100; // 10 Hz serial print rate

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // wait for the USB serial connection

  Serial.println("H3LIS331 test");

  // Make the I2C pin assignment explicit rather than relying on core defaults.
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  if (!lis.begin_I2C(ACCEL_I2C_ADDR)) {
    Serial.println("Could not find H3LIS331 - check wiring and I2C address!");
    while (1) { delay(10); }
  }
  Serial.println("H3LIS331 found");

  // Range options: H3LIS331_RANGE_100_G / _200_G / _400_G
  // 100G gives the finest resolution and is a good starting point for a
  // bench test at rest. Switch to 400G once this is mounted on a robot
  // that will actually spin fast enough to need it.
  lis.setRange(H3LIS331_RANGE_100_G);
  Serial.print("Range set to: ");
  switch (lis.getRange()) {
    case H3LIS331_RANGE_100_G: Serial.println("100 G"); break;
    case H3LIS331_RANGE_200_G: Serial.println("200 G"); break;
    case H3LIS331_RANGE_400_G: Serial.println("400 G"); break;
  }

  lis.setDataRate(LIS331_DATARATE_400_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS331_DATARATE_POWERDOWN:       Serial.println("Powered down"); break;
    case LIS331_DATARATE_50_HZ:           Serial.println("50 Hz"); break;
    case LIS331_DATARATE_100_HZ:          Serial.println("100 Hz"); break;
    case LIS331_DATARATE_400_HZ:          Serial.println("400 Hz"); break;
    case LIS331_DATARATE_1000_HZ:         Serial.println("1000 Hz"); break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ: Serial.println("0.5 Hz (low power)"); break;
    case LIS331_DATARATE_LOWPOWER_1_HZ:   Serial.println("1 Hz (low power)"); break;
    case LIS331_DATARATE_LOWPOWER_2_HZ:   Serial.println("2 Hz (low power)"); break;
    case LIS331_DATARATE_LOWPOWER_5_HZ:   Serial.println("5 Hz (low power)"); break;
    case LIS331_DATARATE_LOWPOWER_10_HZ:  Serial.println("10 Hz (low power)"); break;
  }

  Serial.println();
}

void loop() {
  sensors_event_t event;
  lis.getEvent(&event);

  // acceleration.x/y/z are in m/s^2. At rest, one axis should read close to
  // +/-9.8 m/s^2 (1 g) - that's gravity, and it's the easiest sanity check
  // that wiring and orientation are correct.
  Serial.print("X: ");  Serial.print(event.acceleration.x, 3);
  Serial.print("\tY: "); Serial.print(event.acceleration.y, 3);
  Serial.print("\tZ: "); Serial.print(event.acceleration.z, 3);
  Serial.println(" m/s^2");

  delay(PRINT_INTERVAL_MS);
}
