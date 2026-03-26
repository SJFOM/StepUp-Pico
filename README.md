![Alt Text](./images/StepUp_text_header.svg)
# *StepUp!* - A no-code tool for instant stepper motor testing


<a href="https://hits.dwyl.com/SJFOM/StepUp-Pico"><img src="https://hits.dwyl.com/SJFOM/StepUp-Pico.svg" alt="Hits Badge"/></a>
<a href="https://github.com/SJFOM/StepUp-Pico/stargazers"><img src="https://img.shields.io/github/stars/SJFOM/StepUp-Pico" alt="Stars Badge"/></a>
<a href="https://github.com/SJFOM/StepUp-Pico/network/members"><img src="https://img.shields.io/github/forks/SJFOM/StepUp-Pico" alt="Forks Badge"/></a>
<a href="https://github.com/SJFOM/StepUp-Pico/pulls"><img src="https://img.shields.io/github/issues-pr/SJFOM/StepUp-Pico" alt="Pull Requests Badge"/></a>
<a href="https://github.com/SJFOM/StepUp-Pico/issues"><img src="https://img.shields.io/github/issues/SJFOM/StepUp-Pico" alt="Issues Badge"/></a>
<a href="https://github.com/SJFOM/StepUp-Pico/graphs/contributors"><img alt="GitHub contributors" src="https://img.shields.io/github/contributors/SJFOM/StepUp-Pico?color=2b9348"></a>
<a href="https://github.com/SJFOM/StepUp-Pico/blob/master/LICENSE"><img src="https://img.shields.io/github/license/SJFOM/StepUp-Pico?color=2b9348" alt="License Badge"/></a>


# Hardware
<img src="./images/StepUp_PCB_v0_3.png" alt="PCB" width="800" />


## 3D printed housing assembly

<img src="./images/StepUp_assembly_drawing.png" alt="StepUp exploded assembly view" width="800" />

## Project Structure

```
/StepUp-Pico
|
|___/App-StepUp             // StepUp! Application source code (C++)
|   |___CMakeLists.txt      // Application-level CMake config file
|
|___/Config
|   |___FreeRTOSConfig.h    // FreeRTOS project config file
|
|___/FreeRTOS-Kernel        // FreeRTOS kernel files, included as a submodule
|
|___/images                 // Image files used in the README.md guides
|
|___/Hardware               // Thin layers for global control - used as parent class
|   |___3D print files      // STL files to be 3D printed for StepUp housing
|   |___PCBA files          // Zip file containing Gerbers, Drill files & BoM 
|
|___/Interfaces             // Thin layers for global control - used as parent class
|
|___/Libraries              // Non-HW specific code, commonly used function calls
|
|___CMakeLists.txt          // Top-level project CMake config file
|___pico_sdk_import.cmake   // Raspberry Pi Pico SDK CMake import script
|
|___rp2040.code-workspace   // Visual Studio Code workspace
|
|___README.md
|___LICENSE.md
```


# Prerequisites

## IDEs

This project was developed using Microsoft Visual Studio Code](https://code.visualstudio.com/) and all instructions which follow mention use of its code extensions for working with the StepUp! project. Workspace files are included herein - see [rp2040.code-workspace](./rp2040.code-workspace).


## Quick-start - cloning the repo and uploading code

1. Clone (recursively) the repo: `git clone --recursive https://github.com/SJFOM/StepUp-Pico.git`.
2. Enter the repo: `StepUp-Pico`.
3. Optionally, edit `CMakeLists.txt` and `/App-StepUp/CMakeLists.txt` to configure the project.
4. Optionally, manually configure the build process: `cmake -S . -B build/`.
5. Optionally, manually build the app: `cmake --build build`.
6. Connect your device so it’s ready for file transfer.
7. Copy the `StepUp.uf2` file from the `build/App-StepUp` folder to the drive which represents the attached Pico device hardware.


## VS Code extensions

### Required
Raspberry Pi's own [VSCode extension](https://github.com/raspberrypi/pico-vscode) simplifies the installation process for any OS. It includes other necessary VS Code extensions such as [CMake](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) and [Cortex-Debug](https://github.com/marus/cortex-debug) for both CMake file compilation and debugging using OpenOCD via a SWD capable debugger probe respectively.

### Optional
While the Raspberry Pi Pico extension is the only one _required_ to compile code for this project, an ancilliary list of recommended extensions can be found in the included [`extensions.json`](./.vscode/extensions.json) file and will be prompted for install by the VS Code IDE.


# Uploading code


# Using the device

## Inserting a battery
Only 18650 Li-Ion cells are supported by this device. The 18650 cell can either include battery protection circuitry or not - there is a battery protection circuit on-board the StepUp! device which is configured specifically for the device.

<div style="background-color: #e7f3ff; border-left: 4px solid #0066cc; padding: 10px; margin: 10px 0;">
NOTE: When first inserting the battery, you must also plug in a USB C cable to power up the device. This is a known quirk of the battery protection circuit which prohibits using the battery until external power is first applied.
</div>


## Powering ON
Press and hold the **POWER** button on the side of the device until a the LED flashes GREEN several times and an audible tone of increasing frequencies plays (like a step-up sequence).

## Controlling the Motor
Connect the stepper motor using the connector labelled with `A1, A2, B1, B2` to match the coils of the given motor. The Joystick is used to control the motor direction and speed with the joystick button (click while centered) resetting the motor speed to its default configuration.

<img src="./images/StepUp_fully_assembled_top_view_lines.png" alt="Motor joystick control diagram" width="400" />

By default, the StepUp! device drives the supplied stepper motor with `~500mA` of current. While this _can_ be increased by modifying the firmware (see `DEFAULT_IRUN_VALUE` in [`tmc_control.hpp`](./App-StepUp/include/tmc_control.hpp)) it is not recommended given the limited power available from the provided 18650 battery.

## Status Codes

### `STANDBY` - When not controlling a stepper motor
| Status | Color: Pattern | Buzzer Pattern | Meaning |
|--------|----------------|---------|---------|
| Power ON | Green: Fade, low to high | Sweep, low to high | Device is booting - boot complete once tone completes |
| Ready | Green: Solid | None | Device is ready, LED colour indicates battery HIGH|
| Ready | Orange: Solid | None | Device is ready, LED colour indicates battery MEDIUM|
| Ready | Red: Solid | None | Device is ready, LED colour indicates battery LOW|
| Ready | Red: Blinking | Long sequential beeps | Battery critically LOW - Device auto-powers OFF|

### `ACTIVE` - When controlling a stepper motor
| Status | Color: Pattern | Buzzer Pattern | Meaning |
|--------|----------------|---------|---------|
| Power ON | Green: Fade, low to high | Sweep, low to high | Device is booting - boot complete once tone completes |
| Ready | Green|Yellow|Red: Solid | None | Device is ready, LED colour indicates battery High|Medium|Low |
| Operating | Blue: Blinking | None |Device is operating |

## Powering OFF
Press and hold the **POWER** button on the side of the device until a the LED flashes RED several times and an audible tone of decreasing frequencies plays (like a step-down sequence).

<div style="background-color: #e7f3ff; border-left: 4px solid #0066cc; padding: 10px; margin: 10px 0;">
NOTE: The device will automatically power OFF after 10 minutes of inactivity
</div>

## Charging

# Debugging

<div style="background-color: #e7f3ff; border-left: 4px solid #0066cc; padding: 10px; margin: 10px 0;">
NOTE: The device must first be powered ON for debugging to be enabled.
</div>

While any debug probe offering SWD debugging will work, this project makes use of the very handy [Raspberry Pi Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html) and the [`launch.json`](./.vscode/launch.json) file in this repository is configured as such for use with this probe.

<img src="./images/pico_debug_probe.webp" alt="Pico Debug Probe" width="400" />

A debugging session can be started either through the Raspberry Pi Pico extension or by using the `Run and Debug` extension in the VS Code sidebar. In either case, use the `Pico Debug (Cortex-Debug)` prompt when asked how you want to initalise the session.

<img src="./images/Cortex_Debug_menu.png" alt="Debug menu" width="400" />


## Debugging on MacOS
To get openocd to play ball, you must install the following libraries as described in the (`openocd/README.macOS` file)[https://openocd.org/doc-release/README.macOS]:
```bash
brew install libtool automake libusb libusb-compat hidapi libftdi
```

## Debugging in Linux

### Enable `arm-none-eabi-gdb`
To debug in Linux, you need to ensure you have `arm-none-eabi-gdb` installed. If you can build firmware for the Pico, odds are this is already installed on your PC but not symlinked correctly. To link this, run the following with elevated priveleges (`sudo`)
```shell
ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb
```
You should now see that running `arm-none-eabi-gdb` works as expected.

## Note on use Linux for debugging 

To help Linux to recognise the debug probe, you may need to update your `udev` rules 
By following the very useful steps outlined [here](https://forums.raspberrypi.com/viewtopic.php?t=364698). This will configure your udev rules to recognise the Pico plugged in as a CMSIS-DAP interface.

The relevant steps in the linked guide are as follows:
1. Create a file `10-my-usb.rules` in `/etc/udev/rules.d` containing
    ```
    SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000c", MODE="666", GROUP="plugdev"
    ```
2. Now, restart the udev service using
    ```shell
    sudo udevadm control --reload
    sudo udevadm trigger
    ```

## Credits

This work makes heavy usage of Tony Smith's (a.k.a [smittytone](https://github.com/smittytone)) wonderful [RP2040-FreeRTOS Template](https://github.com/smittytone/RP2040-FreeRTOS) which forms the basic structure for most of this project. Kudos to his work on creating a simple platform to get started with FreeRTOS on the Pi Pico hardware.

## Copyright and Licences

StepUp! application source © 2023, Sam O'Mahony (a.k.a [SJFOM](https://github.com/SJFOM)) and licensed under the terms of the [MIT Licence](./LICENSE.md).

Original template source code © 2022, Tony Smith (a.k.a. [smittytone](https://github.com/smittytone)) and licensed under the terms of the [MIT Licence](./LICENSE.md).

[FreeRTOS](https://freertos.org/) © 2021, Amazon Web Services, Inc. It is also licensed under the terms of the [MIT Licence](./LICENSE.md).

The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) is © 2020, Raspberry Pi (Trading) Ltd. It is licensed under the terms of the [BSD 3-Clause "New" or "Revised" Licence](https://github.com/raspberrypi/pico-sdk/blob/master/LICENSE.TXT).
