# ESP32 HD44780 driver/library

This is a header-only C++ style library (see files in `main/include/`) for driving the popular **HD447870** LCD display from an **ESP32** microcontroller.
Currently, there exists one Connector - using **PCF8574** 8-bit expander via **I2C**.
Feel free to suggest or write other Connectors.

Main source file contains simple tests. You can see the results [here](https://imgur.com/a/MCVgFki).

For C-style library, see [bradkeifer/esp32-HD44780](https://github.com/bradkeifer/esp32-HD44780), on which this library is based.
