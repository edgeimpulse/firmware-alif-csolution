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

## Build
> [!IMPORTANT]
> To build and manage the project, you need to follow this guide on how to setup the [CMSIS-Toolbox](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/README.md)

### Using Arm CMSIS Solution extension
Go to the CMSIS extension tab, and click the hammer icon.

### Using the script
Run the script specifying the target you want to build the project for, if not specified the default is `HP`.

ie to compile HE:
```
sh build.sh HE
```

To clean:
```
sh build.sh clean
```

> [!NOTE]
> On windows, use `build_win.bat` to compile the project.

### Using Docker
TODO

## Flash the board
> [!IMPORTANT]
> To flash the board you need to set the `SETOOLS_ROOT` environmental variable to point to the Alif Security Toolkit

### Flash using VS code

#### Flash using Secure toolkit
Task -> Run Task -> Program with security toolkit.
The actual config will be flashed.

#### Flash using JLink
You can use a Segger JLink to start a debug session and flash the board.
Connect the 20-pin connector to J13 and press F5 to start.

### Flash using script
Run the script specifying the target you want to be flashed, if not specified the default is `HP`.

ie to flash HE:
```
flash.sh HE
```

> [!NOTE]
> On windows, use `flash_win.bat` to flash the target.

## Connect to serial console
To connect to the board, use the following settings:
J15
3-5
4-6
115200, 8, N, 1


## Run a debug session
Connect the 20pin cable to J13 and press F5.

## Update your model
To update your model, unizp the CMSIS pack deployment, install the packs using `cpackget add <your_project_name>.pack` and paste the `model.yml` in the `model` folder.
