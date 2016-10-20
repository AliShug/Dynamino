# Dynamino
Dynamino is a *work-in-progress* C++ library which provides simple, direct control of [Dynamixel](http://en.robotis.com/index/product.php?cate_code=101010) TTL smart servos from 5V ATmega Arduinos. See [here](supported.md) for more information on supported servos and operating modes.

#### What it is
- Dynamino will allow you to enumerate any attached servos, ping them, read data such as their position and temperature, and send them command instructions.
- It's designed to operate at high speed: 1,000,000 baud, which is the default rate for these servos.
- It works with no additional hardware: just the servos, their power supply, and your Arduino. Connect the data line to a digital pin and you're good to go.

#### What it isn't
- It won't and can't work with the RS-485 servos. Sorry.
- By itself, this won't turn your Arduino into a drop-in replacement for a device like the USB2Dynamixel or USB2AX.
- It's *exclusively* an AVR Arduino library. It won't run on your PC, your Raspberry Pi, or really anywhere else. Dynamino relies on low-level assembly code written specifically for ATmega processors, and so also won't run on devices like the Arduino Zero.

## Development
Clone or otherwise download the repo. The core library files are created with a Python-based code-generation process: execute it with `python -m pygen` in the root directory. This (re)generates files in the 'Dynamino' subdirectory.

## Installation
On Windows, the code-generator will attempt to copy the generated library files to the standard Arduino library directory ('%USER%\Documents\Arduino\libraries'). If this doesn't work or if you're on a different platform, simply run the code-generation as above then copy the 'Dynamino' directory to your Arduino libraries directory.

Once installed, you should be able to `#include <Dynamino.h>` and use the library. There's a couple of examples included in the library to get you started.

## License
Dynamino is MIT-licensed, which means you can use it for free (including for commercial purposes) with attribution. It also means that the library comes with absolutely no warranty: you use this software at your own risk. Remember, those servos are expensive, so please do be careful!

See [LICENSE.txt](LICENSE.txt) for the full terms.
