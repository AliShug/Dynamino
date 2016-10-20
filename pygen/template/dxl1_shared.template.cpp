#include "Dynamino.h"

void {{prefix}}_write_1mb(uint8_t data, uint8_t pin)
{
    // 1 Mbaud software serial transmit of 1 byte
    // This disables interrupts for roughly 10us
    volatile uint8_t c, d, p;
    d = data;
    p = 1 << pin;
    asm volatile(
        // Intermediate (clobber) registers
        "on=16 \n"      // Port with output bit HIGH
        "off=17 \n"     // Port with output bit LOW
        "tmp=18 \n"     // Temporary storage in high registers

        // setup
        //"cli \n"

        "in on, 0x0B \n"        // Read/modify port of interest
        "mov off, on \n"
        "or on, %[pin] \n"      // Pin set HIGH
        "ldi tmp, 0xFF \n"
        "eor %[pin], tmp \n"    // NOT pin mask
        "and off, %[pin] \n"    // Pin set LOW
        "ldi %[count], 8 \n"    // 8 bits to send
        "out 0x0B, off \n"      // Set LOW to signal start of byte

        // WAIT 6 cycles
        "nop\n nop\n nop\n nop\n"
        "nop\n nop\n"

    // Loop over 1 byte
    "_%=_asm_loop:\n"
        // WAIT 5 cycles
        "nop\n nop\n nop\n nop\n"
        "nop\n"

        "cpi %[count], 0 \n"
        "breq _%=_end \n"

        // check LSB (shift off right side)
        "lsr %[data] \n"
        "brcs _%=_set \n"
        "nop \n"

        "_%=_clear:\n"
            //"cbi 0x0B, 2 \n"
            "out 0x0B, off \n"
            "rjmp _%=_continue \n"
        "_%=_set:\n"
            //"sbi 0x0B, 2 \n"
            "out 0x0B, on \n"
            "rjmp _%=_continue \n"
        "_%=_continue:\n"
            // loop back around
            "subi %[count], 1 \n"
            "rjmp _%=_asm_loop \n"
    "_%=_end:\n"
        // finish byte
        "nop\n nop\n"
        "out 0x0B, on \n"
        //"sei \n"
        //"nop\n nop\n nop\n nop\n"
        :
        :   [count] "r" (c),
            [data]  "r" (d),
            [pin] "r" (p)
        :   "r16", "r17", "r18");
}

/**
    1 megabaud receive for {{prefix}} status/return packets
 */
uint16_t {{prefix}}_receive_1mb(uint8_t pin, uint8_t params_buff[], uint8_t *nparams)
{
    // Receive buffer
    uint8_t bytes[255];

    // ASM inputs
    volatile uint16_t initial_timeout = 5000;
    volatile uint16_t interim_timeout = 2000;
    volatile uint8_t input_mask = 1 << pin;

    // ASM outputs
    volatile uint8_t receive_count = 0;
    volatile uint8_t limit;

    // Default to 0 params output in case of error
    *nparams = 0;

    // This block of inline assembly performs a rapid poll of the selected input pin,
    // searching for a LOW signalling the start of a serial byte (should be picked up
    // within 500ns of becoming detectable)
    // It then samples the input every 1us, reconstructing the received byte in r18
    // by setting the MSB and right-shifting for each bit received. Bytes are stored
    // in the 'bytes' array one after the other, and control loops back to the start
    // to seek the beginning of the next byte.
    // The fourth (index 3) byte is stored as the expected remaining length ('limit')
    // which allows an early exit (i.e. without a timeout) once the entire packet has
    // been received.
    asm volatile (
        "smp=16 \n"
        "msb=17 \n"
        "byte=18 \n"
        "ldi msb, 1 << 7 \n"
        "ldi %[rec_total], 0 \n"
        "ldi %[limit], 255 \n" // initial limit of 255 bytes

        "cli \n"
        "sbi 0x0B, 3 \n" // 2 TEST
    "_%=_seek_start:\n"
        "in smp, 0x09 \n"       // 1
        "and smp, %[mask] \n"   // 1
        "breq _%=_found \n"        // 1/2 on trigger (found byte start when sample pin is LOW)
        "sbiw %[timeout], 1 \n" // 2
        "breq _%=_rec_end \n"      // 1
        "rjmp _%=_seek_start \n"   // 2
    "_%=_found:\n"
        "cbi 0x0B, 3 \n" // 2 TEST
        // use byte reg to load immediate
        "ldi byte, 8 \n" // 8 bits to collect for byte (bit counter in __TMP_REG__)
        "mov __tmp_reg__, byte \n"
        "ldi byte, 0 \n" // empty byte

        "nop\n nop\n nop\n"
        "nop\n nop\n nop\n nop\n"


    "_%=_rec_bit:\n"
        "lsr byte \n"
        //"sbi 0x0B, 3 \n"
        "in smp, 0x09 \n"
        //"cbi 0x0B, 3 \n"
        "and smp, %[mask] \n"
        "breq _%=_bit_store \n"
        "or byte, msb \n" // set MSB to 1
    "_%=_bit_store:\n"
        "nop\n nop\n nop\n nop\n"
        "dec __tmp_reg__ \n"
        "breq _%=_rec_store \n"
        "nop\n nop\n nop\n"
        "rjmp _%=_rec_bit \n"
    "_%=_rec_store:\n"
        "sbi 0x0b, 3 \n" // 2 DEBUG

        // if it's the 4th (ind 3) byte, set the length limit
        // rec_total is the current index (until incremented)
        "cpi %[rec_total], 3 \n"
        "brne _%=_no_set_length \n"
        "mov %[limit], byte \n"
        "inc %[limit] \n"
    "_%=_no_set_length:\n"

        "st X+, byte \n"
        "inc %[rec_total] \n"
        "movw %[timeout], %[mid_timeout] \n" // reset timeout
        "cbi 0x0B, 3 \n" // 2 DEBUG

        // Decrement the limit - go to end if 0
        "dec %[limit] \n"
        "breq _%=_rec_end \n"
        // Wait (allow line to go HIGH again) then loop to find the next byte
        "nop\n"
        "rjmp _%=_seek_start \n"
    "_%=_rec_end:\n"
        // DEBUG signal end
        "sbi 0x0B, 3 \n"
        "cbi 0x0B, 3 \n"
        "sbi 0x0B, 3 \n"
        "cbi 0x0B, 3 \n"
        "sei \n"

        :   [timeout] "+w" (initial_timeout), // timeout counter MUST be placed in a dedicated register pair
            [rec_total] "+d" (receive_count),
            [limit] "+d" (limit),
            [mask] "+l" (input_mask), // works in lower reg
            [mid_timeout] "+l" (interim_timeout) // reset timeout shoved in a lower register pair

        :   [adr] "x" (bytes)
        :   "r16", "r17", "r18", "memory");

    // If the timeout counter has dropped to zero, we exited without completing the packet
    if (initial_timeout == 0)
    {
        //Serial.print("Received "); Serial.println(receive_count);
        //Serial.print("Limit "); Serial.println(limit);
        if (receive_count > {{prefix}}_INDEX_ERR)
        {
            return {{family}}__ERRFLAGS_TIMEOUT | bytes[{{prefix}}_INDEX_ERR];
        }
        else
        {
            return {{family}}__ERRFLAGS_TIMEOUT;
        }
    }
    else if (receive_count < {{prefix}}_INDEX_LENGTH || receive_count != bytes[{{prefix}}_INDEX_LENGTH] + 4)
    {
        return {{family}}__ERRFLAGS_MALFORMED;
    }
    else
    {
        // Verify checksum
        uint8_t sum = 0;
        for (int i = {{prefix}}_INDEX_ID; i < receive_count - 1; i++)
        {
            sum += bytes[i];
        }
        //Serial.print("SUM "); Serial.println(sum, HEX);
        uint8_t chk = 0xFF ^ sum;
        if (bytes[receive_count - 1] != chk)
        {
            /*Serial.print("Checksum "); Serial.print(bytes[receive_count - 1], HEX);
            Serial.print(" does not match calculated "); Serial.println(chk, HEX);
            for (int i = 0; i < receive_count; i++)
            {
                Serial.print(bytes[i], HEX);
                Serial.print("\n");
            }*/
            return {{family}}__ERRFLAGS_CORRUPT;
        }
        else
        {
            // All good, fill in the parameter(s) length & buffer and return any servo error flags
            if (nparams != NULL && params_buff != NULL)
            {
                *nparams = bytes[{{prefix}}_INDEX_LENGTH] - {{prefix}}_BODY_BASELENGTH;
                memcpy(params_buff, &(bytes[{{prefix}}_INDEX_PARAMS]), *nparams);
            }
            return bytes[{{prefix}}_INDEX_ERR];
        }
    }
}

/**
    Performs a read from memory of an attached servo (EEPROM or RAM)
 */
uint16_t {{prefix}}_read(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t *nparams, uint8_t adr, uint8_t len)
{
    // Send the request
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    noInterrupts();
    {{prefix}}_write_1mb(0xFF, pin);
    {{prefix}}_write_1mb(0xFF, pin);
    {{prefix}}_write_1mb(id, pin);
    {{prefix}}_write_1mb(4, pin);       // base length of 2 + 2 params
    {{prefix}}_write_1mb({{prefix}}_INSTR_READ, pin);
    {{prefix}}_write_1mb(adr, pin);
    {{prefix}}_write_1mb(len, pin);
    {{prefix}}_write_1mb(~(id + 4 + {{prefix}}_INSTR_READ + adr + len), pin);
    pinMode(pin, INPUT);

    // Get the response
    uint16_t status;
    uint8_t attempts = 0;
    do
    {
        status = {{prefix}}_receive_1mb(pin, params_buff, nparams);
        attempts++;
    } while (status & 0xFF00 != 0 && attempts < 3);
    interrupts();

    return status;
}

/**
    Performs a write to memory of an attached servo (EEPROM or RAM)
 */
uint16_t {{prefix}}_write(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t nparams, uint8_t adr)
{
    // Send the request
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    uint8_t len = 3 + nparams; // base length of 2 + address + data to write
    // begin checksum
    uint8_t sum = id + len + {{prefix}}_INSTR_WRITE + adr;
    noInterrupts();
    {{prefix}}_write_1mb(0xFF, pin);
    {{prefix}}_write_1mb(0xFF, pin);
    {{prefix}}_write_1mb(id, pin);
    {{prefix}}_write_1mb(len, pin);
    {{prefix}}_write_1mb({{prefix}}_INSTR_WRITE, pin);
    {{prefix}}_write_1mb(adr, pin);
    for (int i = 0; i < nparams; i++) {
        sum += params_buff[i];
        {{prefix}}_write_1mb(params_buff[i], pin);
    }
    {{prefix}}_write_1mb(~(sum), pin);
    pinMode(pin, INPUT);

    // Get the response
    uint16_t status;
    uint8_t attempts = 0;
    do
    {
        status = {{prefix}}_receive_1mb(pin, NULL, NULL);
        attempts++;
    } while (status & 0xFF00 != 0 && attempts < 3);
    interrupts();

    return status;
}

/**
    Prints a verbose error message to serial for the given error/status code
 */
void {{prefix}}_printErrorMessage(uint16_t errorCode)
{
    if (errorCode == {{family}}__ERRFLAGS_OK)
    {
        Serial.println(F("{{prefix}} No Error ({{family}}__ERRFLAGS_OK)"));
    }
    else
    {
        Serial.println(F("{{prefix}} Error:"));
        if (errorCode & {{family}}__ERRFLAGS_TIMEOUT)
        {
            Serial.println(F("<RECEIVE> TIMEOUT"));
        }
        if (errorCode & {{family}}__ERRFLAGS_CORRUPT)
        {
            Serial.println(F("<RECEIVE> CORRUPTED PACKET (failed checksum)"));
        }
        if (errorCode & {{family}}__ERRFLAGS_MALFORMED)
        {
            Serial.println(F("<RECEIVE> MALFORMED PACKET (unexpected length variation)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_ANGLELIM)
        {
            Serial.println(F("<SEND> ANGLE LIMIT ERROR (goal position out of range)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_CHECKSUM)
        {
            Serial.println(F("<SEND> CORRUPTED PACKET (failed checksum)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_INSTR)
        {
            Serial.println(F("<SEND> INSTRUCTION (undefined or invalid instruction)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_OPRANGE)
        {
            Serial.println(F("<SEND> PARAMETER RANGE (instruction parameter out of range)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_OVERHEAT)
        {
            Serial.println(F("<STATUS> OVERHEAT (internal temperature is outside safe operating range!)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_OVERLOAD)
        {
            Serial.println(F("<STATUS> OVERLOAD (motor loading is outside safe operating range!)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_VOLTAGE)
        {
            Serial.println(F("<STATUS> VOLTAGE (supply voltage outside safe operating range!)"));
        }
        if (errorCode & {{prefix}}_ERRFLAGS_UNDEF)
        {
            Serial.println(F("<STATUS> UNKNOWN (undefined status bit 7 is set)"));
        }
    }
}
