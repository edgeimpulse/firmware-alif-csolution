# firmware-alif-csolution

Firmware for Alif E7 AI/ML AppKit
This application is built on VSCode Getting Started Template (alif_vscode-template)
The default hardware is Gen 2 Ensemble DevKit with camera module and display.

The required software setup consists of VSCode, Git, CMake, cmsis-toolbox, Arm GNU toolchain, Alif tools and JLink (for debugging).
Please refer to the [Getting Started Guide](https://alifsemi.com/download/AUGD0012) for more details.

> [!Note]
> The firmware by default uses the MT9M114 camera sensor, if you have ARX3A0 camera, modify the [firmware-alif.cproject.yaml](firmware-alif.cproject.yml)
> ```
> # - component: AlifSemiconductor::BSP:External peripherals:CAMERA Sensor MT9M114
> - component: AlifSemiconductor::BSP:External peripherals:CAMERA Sensor ARX3A0
> ```
> and the`RTE_Device.h` header for the selected device
> ```
> #define RTE_ARX3A0_CAMERA_SENSOR_CSI_ENABLE             1
> ```


## Prerequisites
1. Create an edge impulse account at [edgeimpulse.com](https://www.edgeimpulse.com/)
2. Install the latest `Alif Security Toolkit`:

    * Navigate to the [Alif Semiconductor Kit documentation](https://alifsemi.com/kits) page (you will need to register to create an account or log in). and download the latest `App Security Toolkit`.
    * Extract archive where you prefer and create an envirionmental variable `SETOOLS_ROOT` pointing to it.
    * Follow the instructions in local the `Alif Security Toolkit Quickstart Guide` to finalize the installation.
3. Clone this repo

### When using `build.sh` on macOS
Building the Alif project requires `nproc` to limit the number of processors used. On macOS, this command is not available. Here you should either install `coreutils` using brew:
```
brew install coreutils
```
Or create an alias for `nproc` that points to the macOS equivalent
```
alias nproc='sysctl -n hw.physicalcpu'
```

## Build
> [!IMPORTANT]
> To build and manage the project, you need to follow this guide on how to setup the [CMSIS-Toolbox](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/README.md)

### Using Arm CMSIS Solution extension
Go to the CMSIS extension tab, and click the hammer icon.

### Using the script
Run the script specifying the target you want to build the project for, if not specified the default is `HP`.
To specify a target, use --target <TARGET> .

ie to compile HE:
```
sh build.sh --target HE
```

Large models that don't fit into `DTCM` memory can be placed in the `SRAM0` section using the `HP_SRAM` argument:
```
sh build.sh --target HP_SRAM
```

To clean:
```
sh build.sh --clean
```

To specify a build config, use --config <BUILD_CONFIG> .
```
sh build.sh --config Speed
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

> [!NOTE]
> When you start a debug session for a specific core, make sure you already flashed the bin for the core using the Security Toolkit, otherwise JLink can't connect to the core.

### Flash using script
Run the script specifying the target you want to be flashed, if not specified the default is `HP`.
To specify a target, use --target <TARGET> .

ie to flash --target HE:
```
flash.sh --target HE
```

To specify a build config, use --config <BUILD_CONFIG>.
```
flash.sh --config Speed
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
To update your model, unizp the CMSIS pack deployment, install the packs using `cpackget add <your_project_name>.pack` and paste the `edgeimpulse.clayer.yml` in the `model` folder.

## Camera config
The firmware by default is using the MT9M114 camera and it initialize it at 320x320 resolution. 
If you need a different one, you have to modify the RTE_device.h for the core you are targeting (HP or HE) and modify this define:

```
// <o> select MT9M114 Image configuration
// <i> Defines camera sensor MT9M114 Image configuration
//     <0=>   1288x728_RAW10
//     <1=>   1280x720_RAW8
//     <2=>   1280x720_RGB565
//     <3=>   640x480_RGB565
//     <4=>   320x240_RGB565
//     <5=>   320x320_RGB565
// <i> Default: 0
#define RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG                5
```
