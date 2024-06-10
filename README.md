# ESP32 MCP4xxx DAC driver/library

This is a single-file header-only C++ style library (see files in `main/include/`) for driving the popular **MCP4xxx** family of MicroChip's analog-digital converters from an **ESP32** microcontroller.

Currently supported are:
- MCP4801, MCP4811, MCP4821
- MCP4802, MCP4812, MCP4822
- MCP4901, MCP4911, MCP4921
- MCP4902, MCP4912, MCP4922
- other versions?


## Installation & usage
See example in `main/main.cpp`.

- Move the files from `main/include/` to your include directory.
- Add `driver` to your `idf_component_register` `REQUIRES`.
- #include the `MCP4xxx.h` in your code.
- Create a custom class or use one of the predefined ones.
- Create its object.
- Done!
