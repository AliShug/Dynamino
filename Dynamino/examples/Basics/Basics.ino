#include <Dynamino.h>

void setup()
{
    Serial.begin(250000);
    while (!Serial);

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
}

void loop()
{
    uint8_t params[255];
    uint8_t nparams;

    uint16_t res = DXL1_read(2, 2, params, &nparams, 0x00, 2);
    if (res == DXL__ERRFLAGS_OK) 
    {
        Serial.print("Model ");
        Serial.println(params[0]);
    }
    else
    {
        DXL1_printErrorMessage(res);
    }
}