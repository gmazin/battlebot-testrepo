/*
  dshot_dual_motor_test.ino

  First-pass bench test: spins two DShot ESCs forward at a fixed 10% throttle
  using bidirectional DShot on an RP2040 (Raspberry Pi Pico / arduino-pico core).

  Library:  bastian2001/pico-bidir-dshot
            Arduino Library Manager name: "Pico_Bidir_DShot"
            https://github.com/bastian2001/pico-bidir-dshot

  Board:    Raspberry Pi Pico (RP2040), via the arduino-pico core
            (Tools > Board > Raspberry Pi RP2040 Boards > Raspberry Pi Pico)

  Wiring:   ESC1 signal -> GPIO MOTOR1_PIN
            ESC2 signal -> GPIO MOTOR2_PIN
            ESC ground  -> Pico GND (common ground with the Pico is required)
            ESC power/motor power comes from your battery/BEC, NOT from the Pico.

  *** SAFETY - READ BEFORE POWERING MOTORS ***
  - Remove props / disengage any weapon, wheel, or drive mechanism before testing.
  - Secure or chock the robot/motor so it can't move or spin unexpectedly.
  - Wire and double-check everything with motor power OFF, then stand clear
    before applying battery power.
  - This sketch has no receiver input and no failsafe - it starts spinning
    automatically ARM_TIME_MS after boot and runs continuously. Don't leave
    it powered and unattended.
  - This is a first draft, not yet verified against real hardware - bench
    test at low throttle (as configured) before trusting it at higher speeds.
*/

#include <PIO_DShot.h>

// ---- Configuration - adjust for your hardware ----
constexpr uint8_t  MOTOR1_PIN   = 2;     // GPIO driving ESC 1 signal wire
constexpr uint8_t  MOTOR2_PIN   = 3;     // GPIO driving ESC 2 signal wire
constexpr uint16_t DSHOT_SPEED  = 600;   // DShot150/300/600/1200 - match your ESC's supported speed

constexpr uint16_t THROTTLE_MAX = 2000;  // library's throttle scale is 0-2000 (0-100%)
constexpr uint16_t THROTTLE_RUN = (THROTTLE_MAX * 10) / 100; // 10% throttle = 200

constexpr uint32_t ARM_TIME_MS  = 3000;  // how long to hold zero-throttle so ESCs arm
constexpr uint32_t SEND_PERIOD_US = 500; // ~2kHz update rate (library wants >500Hz minimum)

BidirDShotX1 *motor1;
BidirDShotX1 *motor2;

void setup() {
  motor1 = new BidirDShotX1(MOTOR1_PIN, DSHOT_SPEED);
  motor2 = new BidirDShotX1(MOTOR2_PIN, DSHOT_SPEED);

  // Most DShot ESCs require a sustained zero-throttle signal before they'll
  // accept a nonzero command (this is the ESC "arming" step). Send zero
  // repeatedly for a few seconds first.
  uint32_t armStart = millis();
  while (millis() - armStart < ARM_TIME_MS) {
    motor1->sendThrottle(0);
    motor2->sendThrottle(0);
    delayMicroseconds(SEND_PERIOD_US);
  }
}

void loop() {
  // The ESC expects a continuous stream of throttle commands - if this stops
  // for too long, the ESC treats it as a lost signal and cuts the motor.
  motor1->sendThrottle(THROTTLE_RUN);
  motor2->sendThrottle(THROTTLE_RUN);
  delayMicroseconds(SEND_PERIOD_US);
}
