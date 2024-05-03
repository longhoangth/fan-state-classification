#include <system.h>

int stt = -1;

enum state
{
    BLADE_FAULT = 0,
    BROKEN_BLADE = 1,
    MOUNT_FALL = 2,
    OFF = 3,
    ON = 4,
    TAP = 5
};

void setup()
{
    Serial.begin(115200);
    setup_tft();
}

void loop()
{
    if (Serial.available())
    {
        stt = Serial.parseInt();
        Serial.println(stt);
    }
    switch (stt)
    {
    case OFF:
        // Serial.println("OFF");
        off();
        break;
    case ON:
        // Serial.println("ON");
        on();
        break;
    case TAP:
        // Serial.println("TAP");
        tapping();
        break;
    case MOUNT_FALL:
        // Serial.println("MOUNT FALL");
        mount_fault();
        break;
    case BLADE_FAULT:
        // Serial.println("BLADE FAULT");
        blade_fault();
        break;
    case BROKEN_BLADE:
        broken_blades();
        break;
    default:
        break;
    }
}