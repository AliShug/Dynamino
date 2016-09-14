#include <Dynamino.h>

void setup()
{
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
}

void loop()
{
    DXL1_write_1mb(0xFF);
}