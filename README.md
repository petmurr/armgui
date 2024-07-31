# armgui README

**To run the windows build, install the latest release as a .zip above. This is v0.6 as of July 2024.**

###  RBE 3001 Robot Arm GUI

Hello! This is the source code for armgui, a simple tool to get familiar with the robot arm we use in RBE 3001. It uses [dear imgui](https://github.com/ocornut/imgui) for the user interface and the [dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/c%2B%2B) to talk to the servos. It uses a state machine to manage program functionality and a custom mouse interaction to control the dials of the program. 

## Directories
- `build/`, for compiled files. this is gitignored.
- `imgui/`, for all imgui source files. includes:
  - `backends/`, for imgui's glfw & opengl3 implementation
- `fonts/` for font files. This must be added manually to a standalone build since it is not included in the binary.
- `include/` for libraries and library header files.
  - `dynamixel_sdk/` dynamixel SDK header files. (all dynamixel files/instructions [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/))
- `src/`, for all source files.
- `utilities/` for the simple bash script that builds a standalone copy of the program so others can download the windows executable. This has not been tested thoroughly.

## Compiling \*WIP*

1. You require a compiler (the Makefile currently uses g++) and GLFW ([glfw.org](http://www.glfw.org)):
- On Linux: <br />
`apt-get install libglfw-dev`
- On Windows (MSYS2): <br />
`pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-glfw`
<br /> <br /> The only library I'm using is the Dynamixel SDK, and it is included with the repo for convenience in `include/`. If you would like to install and use it yourself, platform-specific instructions are here: [emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/). Of course, make sure the makefile can find it.

3. Having cloned `arm_gui/`, build with `make` from the `src/` directory. The executable will .

## OpenmanipulatorX quirks

The openmanipulatorX is a great robot arm, but sometimes there's a hardware error or unexpected behavior that crops up (using armgui or the matlab library). I highly recommend using the [dynamixel wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to see if there's any difference between the hardware table of a working arm and a malfunctioning arm, as well as checking all cables are secured. [Jani has made some great documentation on this](https://github.com/WPI-300x-Lab-Staff).

I recently discovered there is a reproducable problem with the gripper being rotated 180 deg from where it expects itself to be- homing will stutter, and if the matlab script tries to open or close the gripper, dynamixel will throw a hardware error. Try rotating the gripper 180 degrees so it closes "the other way". 