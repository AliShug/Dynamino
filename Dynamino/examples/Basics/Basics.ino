#include <Dynamino.h>

void setup()
{
    Serial.begin(250000);
    while (!Serial);

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);

    uint8_t torqueEnable = 1;
    DXL1_write(2, 3, &torqueEnable, 1, 24);

    uint16_t goalPos = 10;
    DXL1_write(2, 3, (uint8_t*) &goalPos, 2, 30);
}

void loop()
{
    uint8_t params[255];
    uint8_t nparams;

    uint16_t res = DXL1_read(2, 3, params, &nparams, 0x00, 49);
    if (res == DXL__ERRFLAGS_OK) 
    {
        uint16_t *wide_num;

        wide_num = (uint16_t*) &params[0];
        Serial.print("Model ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[6];
        Serial.print("CW angle limit ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[8];
        Serial.print("CCW angle limit ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[14];
        Serial.print("Max torque ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[30];
        Serial.print("Goal position ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[32];
        Serial.print("Moving speed ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[34];
        Serial.print("Torque limit ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[36];
        Serial.print("Present position ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[38];
        Serial.print("Present speed ");
        Serial.println(*wide_num);

        wide_num = (uint16_t*)&params[40];
        Serial.print("Present load ");
        Serial.println(*wide_num);

        Serial.print("Present Voltage ");
        Serial.println(params[42]);

        Serial.print("Moving ");
        Serial.println(params[46]);
    }
    else
    {
        DXL1_printErrorMessage(res);
    }

    delay(100);
}