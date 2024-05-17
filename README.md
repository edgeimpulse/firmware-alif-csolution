# firmware-alif-csolution

Firmware for Alif E7 AI/ML AppKit
This application is built on VSCode Getting Started Template (alif_vscode-template)
The default hardware is Gen 2 Ensemble DevKit with camera module and display.

The required software setup consists of VSCode, Git, CMake, cmsis-toolbox, Arm GNU toolchain, Alif tools and JLink (for debugging).
Please refer to the [Getting Started Guide](https://alifsemi.com/download/AUGD0012) for more details.

## Prerequisites
1. Create an edge impulse account at [edgeimpulse.com](https://www.edgeimpulse.com/)
2. Install the latest `Alif Security Toolkit`:

    * Navigate to the [Alif Semiconductor Kit documentation](https://alifsemi.com/kits) page (you will need to register to create an account or log in). and download the latest `App Security Toolkit`.
    * Extract archive where you prefer and create an envirionmental variable `SETOOLS_ROOT` pointing to it.
    * Follow the instructions in local the `Alif Security Toolkit Quickstart Guide` to finalize the installation.
3. Clone this repo and intitialize the submodules:
```
git submodule init
git submodule update
```

## Configuration
TODO

## Flash the board
To Flash the board, set the jumper J15 in the following position:
1-3
2-4
TODO
Task -> Run Task -> Program with security toolkit

## Connect to serial consolle
To connect to the board, use the following settings:
J15
3-5
4-6
115200, 8, N, 1


## Run a debug session
TODO
Connect the 20pin cable to J13 and press F5.

## Update model
TODO
