# Cube++ Basic Debug Tx/Rx Example (STM32L152)

A bare-bone setup of the Cube++ submodule with a basic Debug task for receiving ascii text commands over a serial terminal using the included UART driver.

Based on the STM32L152, note: requires changes to system clock settings to work with Nucleo boards.

# Contents
1. [Setup](#Setup)

# Setup
- Update submodules
```
git submodule update --init
```
- Import into CubeIDE
  - Select File > Open Projects from File System
