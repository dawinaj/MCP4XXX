# ESP32 MCP3x0x ADC driver/library

This is a single-file header-only C++ style library (see files in `main/include/`) for driving the popular **MCP3xxx** family of MicroChip's analog-digital converters from an **ESP32** microcontroller.

Currently supported are:
- MCP3002, MCP3004, MCP3008
- MCP3202, MCP3204, MCP3208
- MCP3304, MCP3308
- other versions?


It is based on three other libraries:
- https://github.com/bakercp/MCP3XXX
- https://github.com/nopnop2002/esp-idf-mcp3002
- https://github.com/gfurtadoalmeida/esp32-driver-mcp320x


## Installation & usage
See example in `main/main.cpp`.

- Move the files from `main/include/` to your include directory.
- Add `driver` to your `idf_component_register` `REQUIRES`.
- #include the `MCP3x0x.h` in your code.
- Create a custom class or use one of the predefined ones.
- Create its object.
- Done!
