#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define CRSF_BAUDRATE 420000
#define UART_PIN_TX 12
#define UART_PIN_RX 13

// Set up a new Serial object
AlfredoCRSF crsf;

void setup()
{
  Serial.begin(115200);
  Serial1.setTX(UART_PIN_TX);
  Serial1.setRX(UART_PIN_RX);
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);
}

void loop()
{
  crsf.update();
  if (crsf.isLinkUp()) {
    for (int i = 1; i <= 10; i++)
    {
    Serial.print("Ch"+String(i)+": "+crsf.getChannel(i)+"\t");
    }
    Serial.println();
  }
  delay(20);
}