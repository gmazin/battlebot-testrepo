#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <PIO_DShot.h>

#define CRSF_BAUDRATE 420000
#define UART_PIN_TX 12
#define UART_PIN_RX 13

AlfredoCRSF crsf;

// ---- Configuration - adjust for your hardware ----
constexpr uint8_t  MOTOR1_PIN   = 2;     // GPIO driving ESC 1 signal wire
constexpr uint8_t  MOTOR2_PIN   = 3;     // GPIO driving ESC 2 signal wire
constexpr uint16_t DSHOT_SPEED  = 600;   // DShot150/300/600/1200

constexpr uint32_t ARM_TIME_MS  = 3000;  // How long to hold zero-throttle so ESCs arm
constexpr uint32_t SEND_PERIOD_US = 500; // ~2kHz update rate

// Joystick Calibration Constants
const int JOY_MIN = 990;
const int JOY_CTR = 1500;
const int JOY_MAX = 2012;

// Adjustable Parameters
const int DEADZONE = 15;         // Joystick delta from center to ignore
const float EXPONENT = 2.0;      // 1.0 = linear, 2.0 = quadratic curve

BidirDShotX1 *motor1;
BidirDShotX1 *motor2;

// Reusable function to convert any joystick channel to DShot values
int mapJoystickToDShot(int joyIn) {
  // Default to 0 (Stopped) if inside deadzone or if input is invalid
  if (abs(joyIn - JOY_CTR) <= DEADZONE) {
    return 0; 
  }
  
  if (joyIn > JOY_CTR) {
    // Forward Direction
    float inputRange = JOY_MAX - (JOY_CTR + DEADZONE);
    float inputOffset = joyIn - (JOY_CTR + DEADZONE);
    
    // Normalize to 0.0 - 1.0 range and constrain boundaries
    float normalized = inputOffset / inputRange;
    normalized = constrain(normalized, 0.0, 1.0);
    
    // Apply quadratic shaping curve
    float shaped = pow(normalized, EXPONENT);
    
    // Map to ESC forward range: 1001 (min fwd) to 2000 (max fwd)
    return 1001 + round(shaped * 999.0);
  } 
  else {
    // Backward Direction
    float inputRange = (JOY_CTR - DEADZONE) - JOY_MIN;
    float inputOffset = (JOY_CTR - DEADZONE) - joyIn;
    
    // Normalize to 0.0 - 1.0 range and constrain boundaries
    float normalized = inputOffset / inputRange;
    normalized = constrain(normalized, 0.0, 1.0);
    
    // Apply quadratic shaping curve
    float shaped = pow(normalized, EXPONENT);
    
    // Map to ESC backward range: 1 (min back) to 1000 (max back)
    return 1 + round(shaped * 999.0);
  }
}

void setup()
{
  Serial.begin(115200);
  
  // Set up Connection with Receiver
  Serial1.setTX(UART_PIN_TX);
  Serial1.setRX(UART_PIN_RX);
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  motor1 = new BidirDShotX1(MOTOR1_PIN, DSHOT_SPEED);
  motor2 = new BidirDShotX1(MOTOR2_PIN, DSHOT_SPEED);

  // Arming sequence: send 0 (stopped) during startup
  uint32_t armStart = millis();
  while (millis() - armStart < ARM_TIME_MS) {
    motor1->sendThrottle(0);
    motor2->sendThrottle(0);
    delayMicroseconds(SEND_PERIOD_US);
  }
}

void loop()
{
  crsf.update();
  
  if (crsf.isLinkUp()) {
    int rawJoy1 = crsf.getChannel(2);
    int rawJoy2 = crsf.getChannel(3);

    // Iterate the calculation function across your inputs
    int motor1Speed = mapJoystickToDShot(rawJoy1);
    int motor2Speed = mapJoystickToDShot(rawJoy2);
    
    // Output telemetry
    Serial.print("M1 In: "); Serial.print(rawJoy1);
    Serial.print(" -> Out: "); Serial.print(motor1Speed);
    Serial.print(" | M2 In: "); Serial.print(rawJoy2);
    Serial.print(" -> Out: "); Serial.println(motor2Speed);
    
    // Command the motors independently
    motor1->sendThrottle(motor1Speed);
    motor2->sendThrottle(motor2Speed);
    delayMicroseconds(SEND_PERIOD_US);
  }
  else {
    // Failsafe behavior: Stop all motors safely if connection drops
    motor1->sendThrottle(0);
    motor2->sendThrottle(0);
  }
}