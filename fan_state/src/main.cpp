#include <Arduino.h>
#include <HardwareSerial.h>

#define TX 17
#define RX 18

HardwareSerial uart1(1);

int state = 0;

void setup()
{
  uart1.begin(115200, SERIAL_8N1, RX, TX);
}
void loop()
{
  if (state > 6)
  {
    state = 0;
  }
  uart1.println(state);
  state++;
  delay(1000);
}