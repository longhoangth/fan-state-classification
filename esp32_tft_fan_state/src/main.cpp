#include <system.h>

int curr_state = -1;
int prev_state = -1;
extern int count;
extern TFT_eSPI tft;

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
        curr_state = Serial.parseInt();
    }
    if (curr_state != prev_state)
    {
        switch (curr_state)
        {
        case OFF:
            off();
            break;
        case ON:
            on();
            break;
        case TAP:
            tapping();
            break;
        case MOUNT_FALL:
            mount_fault();
            break;
        case BLADE_FAULT:
            blade_fault();
            break;
        case BROKEN_BLADE:
            broken_blades();
            break;
        default:
            break;
        }
        prev_state = curr_state;
    }

    if (curr_state == BLADE_FAULT || curr_state == BROKEN_BLADE || curr_state == MOUNT_FALL)
    {
        if (count % 2 == 0)
        {
            tft.fillRect(0, 50, 240, 10, TFT_RED);
            tft.fillRect(0, 50, 10, 320, TFT_RED);
            tft.fillRect(230, 50, 10, 320, TFT_RED);
            tft.fillRect(0, 310, 240, 10, TFT_RED);
        }
        else
        {
            tft.fillRect(0, 50, 240, 10, TFT_WHITE);
            tft.fillRect(0, 50, 10, 320, TFT_WHITE);
            tft.fillRect(230, 50, 10, 320, TFT_WHITE);
            tft.fillRect(0, 310, 240, 10, TFT_WHITE);
        }
    }
}